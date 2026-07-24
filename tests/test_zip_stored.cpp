// Direct tests for zip_stored.hpp -- writeStoredZip()'s own STORED-entry
// round trip, plus reading a DEFLATE-compressed (method 8) entry, which is
// what most third-party ZIP/3MF writers actually emit.

#include "openscad_cpp_evaluator/zip_stored.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

using namespace oscadeval;

namespace {

std::filesystem::path tempPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("oscad_eval_test_" + name);
}

template <typename T>
void putLE(std::vector<uint8_t>& out, T v) {
    for (size_t i = 0; i < sizeof(T); ++i) out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}

// Hand-builds a single-entry ZIP archive with an explicit compression
// method, bypassing writeStoredZip() (which only ever writes method 0) so
// DEFLATE (method 8) reading can be exercised without a second ZIP-writing
// dependency.
void writeSingleEntryZip(const std::string& path, const std::string& name, uint16_t method, uint32_t crc,
                          uint32_t uncompressedSize, const std::vector<uint8_t>& compressedData) {
    std::vector<uint8_t> out;
    const uint32_t offset = 0;

    putLE<uint32_t>(out, 0x04034b50u);
    putLE<uint16_t>(out, 20);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, method);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, 0);
    putLE<uint32_t>(out, crc);
    putLE<uint32_t>(out, static_cast<uint32_t>(compressedData.size()));
    putLE<uint32_t>(out, uncompressedSize);
    putLE<uint16_t>(out, static_cast<uint16_t>(name.size()));
    putLE<uint16_t>(out, 0);
    out.insert(out.end(), name.begin(), name.end());
    out.insert(out.end(), compressedData.begin(), compressedData.end());

    const uint32_t cdStart = static_cast<uint32_t>(out.size());
    putLE<uint32_t>(out, 0x02014b50u);
    putLE<uint16_t>(out, 20);
    putLE<uint16_t>(out, 20);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, method);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, 0);
    putLE<uint32_t>(out, crc);
    putLE<uint32_t>(out, static_cast<uint32_t>(compressedData.size()));
    putLE<uint32_t>(out, uncompressedSize);
    putLE<uint16_t>(out, static_cast<uint16_t>(name.size()));
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, 0);
    putLE<uint32_t>(out, 0);
    putLE<uint32_t>(out, offset);
    out.insert(out.end(), name.begin(), name.end());
    const uint32_t cdSize = static_cast<uint32_t>(out.size()) - cdStart;

    putLE<uint32_t>(out, 0x06054b50u);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, 0);
    putLE<uint16_t>(out, 1);
    putLE<uint16_t>(out, 1);
    putLE<uint32_t>(out, cdSize);
    putLE<uint32_t>(out, cdStart);
    putLE<uint16_t>(out, 0);

    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
}

} // namespace

TEST(ZipStored, StoredRoundTrip) {
    const auto path = tempPath("stored.zip");
    const std::string content = "hello stored world";
    writeStoredZip(path.string(), {ZipEntry{"hello.txt", std::vector<uint8_t>(content.begin(), content.end())}});

    const std::vector<uint8_t> read = readStoredZipEntry(path.string(), "hello.txt");
    EXPECT_EQ(std::string(read.begin(), read.end()), content);
    std::filesystem::remove(path);
}

TEST(ZipStored, ReadsDeflateCompressedEntry) {
    // Raw DEFLATE bytes (zlib wbits=-15, i.e. no header/trailer -- ZIP's
    // own "method 8" stream format) for the 51-byte payload below,
    // precomputed via Python's zlib.compressobj(9, DEFLATED, -15).
    const std::string plaintext = "Hello, DEFLATE ZIP world! Hello, DEFLATE ZIP world!";
    const std::vector<uint8_t> compressed = {
        0xf3, 0x48, 0xcd, 0xc9, 0xc9, 0xd7, 0x51, 0x70, 0x71, 0x75, 0xf3, 0x71, 0x0c, 0x71,
        0x55, 0x88, 0xf2, 0x0c, 0x50, 0x28, 0xcf, 0x2f, 0xca, 0x49, 0x51, 0x54, 0xf0, 0xc0,
        0x25, 0x03, 0x00,
    };
    ASSERT_EQ(plaintext.size(), 51u);

    const auto path = tempPath("deflate.zip");
    writeSingleEntryZip(path.string(), "hello.txt", /*method=*/8, /*crc=*/0xbde53822u,
                        static_cast<uint32_t>(plaintext.size()), compressed);

    const std::vector<uint8_t> read = readStoredZipEntry(path.string(), "hello.txt");
    EXPECT_EQ(std::string(read.begin(), read.end()), plaintext);
    std::filesystem::remove(path);
}

TEST(ZipStored, WriteDeflateRoundTrip) {
    // Highly repetitive text compresses well -- also exercises reading our
    // own DEFLATE output back through readStoredZipEntry's method-8 path,
    // a full write->read round trip through this project's own code both
    // directions.
    const std::string content(500, 'a');
    const auto p = tempPath("deflate_roundtrip.zip");
    writeDeflateZip(p.string(), {ZipEntry{"hello.txt", std::vector<uint8_t>(content.begin(), content.end())}});
    const std::vector<uint8_t> read = readStoredZipEntry(p.string(), "hello.txt");
    EXPECT_EQ(std::string(read.begin(), read.end()), content);
    std::filesystem::remove(p);
}

TEST(ZipStored, WriteDeflateActuallyShrinksCompressibleData) {
    const std::string content(2000, 'x');
    const auto p = tempPath("deflate_shrink.zip");
    writeDeflateZip(p.string(), {ZipEntry{"big.txt", std::vector<uint8_t>(content.begin(), content.end())}});
    EXPECT_LT(std::filesystem::file_size(p), content.size());

    const std::vector<uint8_t> read = readStoredZipEntry(p.string(), "big.txt");
    EXPECT_EQ(std::string(read.begin(), read.end()), content);
    std::filesystem::remove(p);
}

TEST(ZipStored, WriteDeflateFallsBackToStoredForTinyEntry) {
    // A handful of bytes can't shrink under DEFLATE once its own framing
    // overhead is counted -- writeDeflateZip should store it raw instead
    // of writing a larger "compressed" entry.
    const std::string content = "hi";
    const auto p = tempPath("deflate_tiny.zip");
    writeDeflateZip(p.string(), {ZipEntry{"tiny.txt", std::vector<uint8_t>(content.begin(), content.end())}});
    const std::vector<uint8_t> read = readStoredZipEntry(p.string(), "tiny.txt");
    EXPECT_EQ(std::string(read.begin(), read.end()), content);
    std::filesystem::remove(p);
}

TEST(ZipStored, WriteDeflateMultipleEntries) {
    const auto p = tempPath("deflate_multi.zip");
    const std::string a(300, 'a'), b(300, 'b');
    writeDeflateZip(p.string(), {
                                     ZipEntry{"a.txt", std::vector<uint8_t>(a.begin(), a.end())},
                                     ZipEntry{"b.txt", std::vector<uint8_t>(b.begin(), b.end())},
                                 });
    const std::vector<uint8_t> readA = readStoredZipEntry(p.string(), "a.txt");
    const std::vector<uint8_t> readB = readStoredZipEntry(p.string(), "b.txt");
    EXPECT_EQ(std::string(readA.begin(), readA.end()), a);
    EXPECT_EQ(std::string(readB.begin(), readB.end()), b);
    std::filesystem::remove(p);
}

TEST(ZipStored, UnsupportedCompressionMethodThrows) {
    const auto path = tempPath("bogus_method.zip");
    // Method 12 (BZIP2) -- real but genuinely unsupported here.
    writeSingleEntryZip(path.string(), "hello.txt", /*method=*/12, /*crc=*/0, /*uncompressedSize=*/0, {});
    EXPECT_THROW(readStoredZipEntry(path.string(), "hello.txt"), std::runtime_error);
    std::filesystem::remove(path);
}
