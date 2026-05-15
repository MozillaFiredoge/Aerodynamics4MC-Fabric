@file:Suppress("unused", "DuplicatedCode")

import dev.kikugie.fletching_table.extension.FletchingTableExtension
import dev.kikugie.stonecutter.StonecutterExperimentalAPI
import dev.kikugie.stonecutter.build.StonecutterBuildExtension
import org.gradle.api.DefaultTask
import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.api.artifacts.dsl.RepositoryHandler
import org.gradle.api.artifacts.repositories.MavenArtifactRepository
import org.gradle.api.file.RegularFileProperty
import org.gradle.api.plugins.JavaPluginExtension
import org.gradle.api.provider.Property
import org.gradle.api.tasks.Copy
import org.gradle.api.tasks.Input
import org.gradle.api.tasks.OutputFile
import org.gradle.api.tasks.TaskAction
import org.gradle.internal.extensions.stdlib.toDefaultLowerCase
import org.gradle.jvm.tasks.Jar
import org.gradle.kotlin.dsl.*
import org.gradle.language.jvm.tasks.ProcessResources
import org.gradle.plugins.ide.idea.model.IdeaModel
import java.io.File
import javax.inject.Inject

val Project.sc: StonecutterBuildExtension
	get() = extensions.getByType<StonecutterBuildExtension>()

@OptIn(StonecutterExperimentalAPI::class)
fun Project.prop(name: String): String = (project.sc.properties.get<String>(name))

fun Project.env(variable: String): String? = providers.environmentVariable(variable).orNull

fun Project.envTrue(variable: String): Boolean = env(variable)?.toDefaultLowerCase() == "true"

fun RepositoryHandler.strictMaven(
	url: String, vararg groups: String, configure: MavenArtifactRepository.() -> Unit = {}
) = exclusiveContent {
	forRepository { maven(url) { configure() } }
	filter { groups.forEach(::includeGroup) }
}

abstract class GenerateModManifestTask : DefaultTask() {
	@get:Input
	abstract val content: Property<String>

	@get:OutputFile
	abstract val outputFile: RegularFileProperty

	@TaskAction
	fun generate() {
		val file = outputFile.get().asFile
		file.parentFile.mkdirs()
		file.writeText(content.get())
	}
}

abstract class ModPlatformPlugin @Inject constructor() : Plugin<Project> {
	override fun apply(project: Project) = with(project) {
		val inferredLoader = Loader.of(project.buildFile.name.substringAfter('.').replace(".gradle.kts", ""))

		val extension = extensions.create("platform", ModPlatformExtension::class.java).apply {
			loader.convention(inferredLoader.id)
			jarTask.convention(inferredLoader.jarTask)
			sourcesJarTask.convention(inferredLoader.sourcesJarTask)
		}

		extensions.create("mixins", MixinsExtension::class.java)

		listOf("org.jetbrains.kotlin.jvm", "com.google.devtools.ksp", "dev.kikugie.fletching-table").forEach {
			apply(
				plugin = it
			)
		}

		afterEvaluate {
			val ctx = Context(
				project = this,
				extension = extension,
				loader = Loader.of(extension.loader.get()),
				stonecutter = project.sc
			)
			configureProject(ctx)
		}
	}

	private fun Project.configureProject(ctx: Context) {
		listOf("java", "me.modmuss50.mod-publish-plugin", "idea").forEach { apply(plugin = it) }

		version = ctx.fullVersion
		ctx.extension.requiredJava.set(ctx.javaVersion)

		if (ctx.loader.isFabricLike) {
			ctx.extension.dependencies {
				required("java") { fabricLikeVersionRange = ">=${ctx.javaVersion.majorVersion}" }
			}
		}

		configureFletchingTable(ctx)
		registerGenerateManifestTask(ctx)
		configureJarTask(ctx)
		configureIdea()
		configureProcessResources(ctx)
		configureJava(ctx)
		configureNativeResources(ctx)
		registerBuildAndCollectTask(ctx)

		configureModPublishing(ctx)

		if (envTrue("PUB_MAVEN_ENABLE")) {
			configureMavenPublishing(ctx)
		}
	}

	private fun Project.configureJava(ctx: Context) {
		extensions.configure<JavaPluginExtension>("java") {
			withSourcesJar()
			withJavadocJar()
			sourceCompatibility = ctx.javaVersion
			targetCompatibility = ctx.javaVersion
		}
	}

	private fun Project.registerGenerateManifestTask(ctx: Context) {
		val manifestOutputDir = layout.buildDirectory.dir("generated/modManifest")
		val generateTask = tasks.register<GenerateModManifestTask>("generateModManifest") {
			content.set(ctx.loader.generateManifest(ctx))
			outputFile.set(layout.buildDirectory.file("generated/modManifest/${ctx.loader.modManifestPath}"))
		}

		the<JavaPluginExtension>().sourceSets.named("main") { resources.srcDir(manifestOutputDir) }
		tasks.named<ProcessResources>("processResources") { dependsOn(generateTask) }
		tasks.withType<Jar>().configureEach {
			if (name == ctx.loader.sourcesJarTask) {
				dependsOn(generateTask)
			}
		}
	}

	@Suppress("UnstableApiUsage")
	private fun Project.configureProcessResources(ctx: Context) {
		tasks.named<ProcessResources>("processResources") {
			dependsOn(tasks.named("stonecutterGenerate"), "kspKotlin")

			val mcVersion = ctx.stonecutter.current.version.split("-")[0]
			val mixinsExt = project.extensions.findByType<MixinsExtension>()

			if (mixinsExt != null && mixinsExt.hasAnyMixins()) {
				val commonMixins = resolveMixinsForVersion(mixinsExt.common, mcVersion, ctx.stonecutter)
				val clientMixins = resolveMixinsForVersion(mixinsExt.client, mcVersion, ctx.stonecutter)
				val serverMixins = resolveMixinsForVersion(mixinsExt.server, mcVersion, ctx.stonecutter)

				val commonArray = commonMixins.toMixinJsonArray()
				val clientArray = clientMixins.toMixinJsonArray()
				val serverArray = serverMixins.toMixinJsonArray()

				logMixinConfiguration(
					logger = project.logger,
					mcVersion = mcVersion,
					commonCount = commonMixins.size,
					clientCount = clientMixins.size,
					serverCount = serverMixins.size
				)

				processMixinFiles(ctx, mapOf(
					"java" to "JAVA_${ctx.javaVersion.majorVersion}",
					"common_array" to commonArray,
					"client_array" to clientArray,
					"server_array" to serverArray
				))

				inputs.property("mcVersion", mcVersion)
				inputs.property("commonMixins", commonArray)
				inputs.property("clientMixins", clientArray)
				inputs.property("serverMixins", serverArray)
			} else {
				processMixinFiles(ctx, mapOf(
					"java" to "JAVA_${ctx.javaVersion.majorVersion}"
				))
			}

			exclude(ctx.loader.excludedResources)
		}
	}

	private fun ProcessResources.processMixinFiles(ctx: Context, expansionMap: Map<String, String>) {
		filesMatching("*.mixins.json") {
			expand(expansionMap)

			if (ctx.loader is Loader.Forge) {
				filter { line ->
					if (line.contains("\"package\"") && line.trim().endsWith(",")) {
						line + "\n    \"refmap\": \"${ctx.modId}.mixins.refmap.json\","
					} else {
						line
					}
				}
			}
		}
	}

	private fun Project.configureNativeResources(ctx: Context) {
		val generatedNativeResourcesDir = layout.buildDirectory.dir("generated/native-resources")

		val prepareNativeResources = tasks.register("prepareNativeResources") {
			outputs.dir(generatedNativeResourcesDir)
			outputs.upToDateWhen { false }

			doLast {
				val outDir = generatedNativeResourcesDir.get().asFile
				outDir.deleteRecursively()
				outDir.mkdirs()

				var packedCount = 0

				val distNativesDir = file("native/dist/natives")
				if (distNativesDir.exists()) {
					copy {
						from(distNativesDir)
						into(outDir.resolve("natives"))
					}
					packedCount += fileTree(distNativesDir).files.size
				}

				val prebuiltDir = file("native/prebuilt")
				if (prebuiltDir.exists()) {
					copy {
						from(prebuiltDir)
						into(outDir)
						exclude("**/.gitkeep")
					}
					packedCount += fileTree(prebuiltDir) { exclude("**/.gitkeep") }.files.size
				}

				val copyNativeBinary = { src: File, relativeTarget: String ->
					if (src.exists()) {
						val dst = outDir.resolve(relativeTarget)
						dst.parentFile.mkdirs()
						dst.writeBytes(src.readBytes())
						packedCount++
					}
				}

				copyNativeBinary(file("native/build/libaero_lbm.so"), "natives/linux-x86_64/libaero_lbm.so")
				copyNativeBinary(file("native/build/aero_lbm.dll"), "natives/windows-x86_64/aero_lbm.dll")
				copyNativeBinary(file("native/build/Release/aero_lbm.dll"), "natives/windows-x86_64/aero_lbm.dll")
				copyNativeBinary(file("native/build-linux-arm64/libaero_lbm.so"), "natives/linux-arm64/libaero_lbm.so")
				copyNativeBinary(file("native/build-macos-arm64/libaero_lbm.dylib"), "natives/macos-arm64/libaero_lbm.dylib")
				copyNativeBinary(file("native/build-windows-x86_64/aero_lbm.dll"), "natives/windows-x86_64/aero_lbm.dll")

				if (packedCount == 0) {
					logger.lifecycle("prepareNativeResources: no native binaries found")
				} else {
					logger.lifecycle("prepareNativeResources: packaged $packedCount native file(s)")
				}
			}
		}

		tasks.named<ProcessResources>("processResources") {
			dependsOn(prepareNativeResources)
			from(generatedNativeResourcesDir)
		}

		tasks.withType<Jar>().configureEach {
			if (name == ctx.loader.sourcesJarTask) {
				dependsOn(prepareNativeResources)
			}
		}
	}

	private fun Project.configureJarTask(ctx: Context) {
		val generateTask = tasks.named("generateModManifest")
		tasks.withType<Jar>().configureEach {
			archiveBaseName.set(ctx.modId)
			dependsOn(generateTask)
			if (ctx.loader is Loader.Forge) {
				manifest.attributes(ctx.loader.mixinConfigAttribute to "${ctx.modId}.mixins.json")
			}
		}
	}

	private fun Project.configureIdea() {
		extensions.configure<IdeaModel>("idea") {
			module {
				isDownloadJavadoc = true
				isDownloadSources = true
			}
		}
	}

	private fun Project.configureFletchingTable(ctx: Context) {
		extensions.configure<FletchingTableExtension> {
			mixins.create("main") { mixin("default", "${ctx.modId}.mixins.json") }
			j52j.register("main") { extension("json", "**/*.json5") }
		}
	}

	private fun Project.registerBuildAndCollectTask(ctx: Context) {
		tasks.register<Copy>("buildAndCollect") {
			from(
				tasks.named(ctx.extension.jarTask.get()),
				tasks.named(ctx.extension.sourcesJarTask.get()),
				tasks.named("javadocJar")
			)
			into(rootProject.layout.buildDirectory.file("libs/${ctx.basicVersion}"))
			dependsOn("build")
		}
	}
}
