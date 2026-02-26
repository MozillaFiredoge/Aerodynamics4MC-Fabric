package com.aerodynamics4mc.client;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HexFormat;
import java.util.Locale;

final class NativeLibraryLoader {
    private static final String RESOURCE_ROOT = "/natives";

    private NativeLibraryLoader() {
    }

    static void loadBundled(String libraryBaseName) throws IOException {
        String os = detectOs();
        String arch = detectArch();
        if (os.equals("windows") && !arch.equals("x86_64")) {
            throw new IOException("Unsupported Windows arch for native solver: " + arch + " (only x86_64)");
        }
        String fileName = nativeFileName(os, libraryBaseName);
        String resourcePath = RESOURCE_ROOT + "/" + os + "-" + arch + "/" + fileName;

        byte[] bytes;
        try (InputStream in = NativeLibraryLoader.class.getResourceAsStream(resourcePath)) {
            if (in == null) {
                throw new IOException("Native library missing in mod jar: " + resourcePath);
            }
            bytes = in.readAllBytes();
        }

        Path targetDir = Path.of(System.getProperty("java.io.tmpdir"), "aerodynamics4mc", "natives");
        Files.createDirectories(targetDir);

        String hash = shortSha256(bytes);
        String extractedName = appendHashToFilename(fileName, hash);
        Path extractedPath = targetDir.resolve(extractedName);

        if (!Files.exists(extractedPath)) {
            Path tmp = Files.createTempFile(targetDir, extractedName + ".", ".tmp");
            Files.write(tmp, bytes);
            try {
                Files.move(tmp, extractedPath, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
            } catch (AtomicMoveNotSupportedException ignored) {
                Files.move(tmp, extractedPath, StandardCopyOption.REPLACE_EXISTING);
            }
        }
        extractedPath.toFile().deleteOnExit();
        System.load(extractedPath.toAbsolutePath().toString());
    }

    private static String detectOs() {
        String os = System.getProperty("os.name", "").toLowerCase(Locale.ROOT);
        if (os.contains("win")) {
            return "windows";
        }
        if (os.contains("mac") || os.contains("darwin")) {
            return "macos";
        }
        if (os.contains("linux")) {
            return "linux";
        }
        throw new IllegalStateException("Unsupported OS for native solver: " + os);
    }

    private static String detectArch() {
        String arch = System.getProperty("os.arch", "").toLowerCase(Locale.ROOT);
        if (arch.equals("x86_64") || arch.equals("amd64")) {
            return "x86_64";
        }
        if (arch.equals("aarch64") || arch.equals("arm64")) {
            return "arm64";
        }
        throw new IllegalStateException("Unsupported CPU arch for native solver: " + arch);
    }

    private static String nativeFileName(String os, String libraryBaseName) {
        if (os.equals("windows")) {
            return libraryBaseName + ".dll";
        }
        if (os.equals("macos")) {
            return "lib" + libraryBaseName + ".dylib";
        }
        return "lib" + libraryBaseName + ".so";
    }

    private static String shortSha256(byte[] bytes) throws IOException {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            byte[] hash = digest.digest(bytes);
            return HexFormat.of().formatHex(hash, 0, 8);
        } catch (NoSuchAlgorithmException e) {
            throw new IOException("SHA-256 not available", e);
        }
    }

    private static String appendHashToFilename(String fileName, String hash) {
        int dot = fileName.lastIndexOf('.');
        if (dot <= 0) {
            return fileName + "-" + hash;
        }
        String base = fileName.substring(0, dot);
        String ext = fileName.substring(dot);
        return base + "-" + hash + ext;
    }
}
