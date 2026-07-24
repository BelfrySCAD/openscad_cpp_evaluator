#include "openscad_cpp_evaluator/zip_stored.hpp"

// Declarations only (STB_IMAGE_IMPLEMENTATION is defined once, in
// surface_load.cpp) -- reuses stb_image's own raw-DEFLATE inflate
// (stbi_zlib_decode_noheader_*, used there for PNG chunk decompression) to
// read DEFLATE-compressed ZIP entries, since ZIP's "method 8" stream is
// exactly that: a raw DEFLATE stream with no zlib/gzip wrapper. Already a
// project dependency (PNG heightmaps in surface()) -- no new library needed.
#include <stb_image.h>

// STB_IMAGE_WRITE_IMPLEMENTATION defined once, here -- this is the only
// translation unit in the library that needs stb_image_write's actual
// symbols (writeDeflateZip's compressor below); tests/test_surface.cpp
// includes the header too (for its PNG fixture writer) but must NOT also
// define the implementation macro, or both object files linked into the
// test binary would define the same symbols twice (ODR violation).
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace oscadeval {

namespace {

uint32_t crc32(const uint8_t* data, size_t len) {
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            t[i] = c;
        }
        return t;
    }();
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

template <typename T>
void putLE(std::vector<uint8_t>& out, T v) {
    for (size_t i = 0; i < sizeof(T); ++i) out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}

uint16_t readU16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

// A fixed, valid DOS date/time (2020-01-01 00:00:00) -- the exact instant
// doesn't matter for a generated 3MF, just that it's a well-formed DOS
// timestamp rather than the all-zero sentinel some tools treat specially.
constexpr uint16_t kDosTime = 0;
constexpr uint16_t kDosDate = (40u << 9) | (1u << 5) | 1u;

// Compresses `data` to a raw DEFLATE stream (no zlib/gzip wrapper) -- what
// ZIP's "method 8" entries want. stbi_zlib_compress() (stb_image_write.h,
// already vendored here for surface()'s PNG writing needs elsewhere in this
// project) produces a full zlib stream instead: a 2-byte header, the same
// raw DEFLATE data in the middle, then a 4-byte Adler-32 trailer -- so
// stripping both ends gives exactly the raw stream ZIP wants. Returns an
// empty vector if compression fails (caller falls back to STORED) or
// doesn't actually shrink `data`.
std::vector<uint8_t> deflateRaw(const std::vector<uint8_t>& data) {
    int outLen = 0;
    unsigned char* compressed = stbi_zlib_compress(const_cast<unsigned char*>(data.data()),
                                                     static_cast<int>(data.size()), &outLen, /*quality=*/8);
    if (!compressed) return {};
    std::vector<uint8_t> raw;
    if (outLen > 6) raw.assign(compressed + 2, compressed + outLen - 4);
    free(compressed);
    return raw;
}

// One entry ready to place into the archive -- writeStoredZip()/
// writeDeflateZip() both just build a list of these and hand it to
// writeZipEntries(), which knows nothing about compression itself.
struct PlacedEntry {
    std::string name;
    uint16_t method;
    std::vector<uint8_t> data; // already compressed if method == 8
    uint32_t crc;
    uint32_t uncompressedSize;
};

void writeZipEntries(const std::string& path, const std::vector<PlacedEntry>& entries) {
    std::vector<uint8_t> out;
    std::vector<uint32_t> offsets;
    offsets.reserve(entries.size());

    for (const auto& e : entries) {
        offsets.push_back(static_cast<uint32_t>(out.size()));
        const uint32_t compressedSize = static_cast<uint32_t>(e.data.size());

        putLE<uint32_t>(out, 0x04034b50u);
        putLE<uint16_t>(out, 20); // version needed
        putLE<uint16_t>(out, 0);  // flags
        putLE<uint16_t>(out, e.method);
        putLE<uint16_t>(out, kDosTime);
        putLE<uint16_t>(out, kDosDate);
        putLE<uint32_t>(out, e.crc);
        putLE<uint32_t>(out, compressedSize);
        putLE<uint32_t>(out, e.uncompressedSize);
        putLE<uint16_t>(out, static_cast<uint16_t>(e.name.size()));
        putLE<uint16_t>(out, 0); // extra field length
        out.insert(out.end(), e.name.begin(), e.name.end());
        out.insert(out.end(), e.data.begin(), e.data.end());
    }

    const uint32_t cdStart = static_cast<uint32_t>(out.size());
    for (size_t i = 0; i < entries.size(); ++i) {
        const auto& e = entries[i];
        const uint32_t compressedSize = static_cast<uint32_t>(e.data.size());

        putLE<uint32_t>(out, 0x02014b50u);
        putLE<uint16_t>(out, 20); // version made by
        putLE<uint16_t>(out, 20); // version needed
        putLE<uint16_t>(out, 0);  // flags
        putLE<uint16_t>(out, e.method);
        putLE<uint16_t>(out, kDosTime);
        putLE<uint16_t>(out, kDosDate);
        putLE<uint32_t>(out, e.crc);
        putLE<uint32_t>(out, compressedSize);
        putLE<uint32_t>(out, e.uncompressedSize);
        putLE<uint16_t>(out, static_cast<uint16_t>(e.name.size()));
        putLE<uint16_t>(out, 0); // extra field length
        putLE<uint16_t>(out, 0); // comment length
        putLE<uint16_t>(out, 0); // disk number start
        putLE<uint16_t>(out, 0); // internal attrs
        putLE<uint32_t>(out, 0); // external attrs
        putLE<uint32_t>(out, offsets[i]);
        out.insert(out.end(), e.name.begin(), e.name.end());
    }
    const uint32_t cdSize = static_cast<uint32_t>(out.size()) - cdStart;

    putLE<uint32_t>(out, 0x06054b50u);
    putLE<uint16_t>(out, 0); // disk number
    putLE<uint16_t>(out, 0); // disk with CD
    putLE<uint16_t>(out, static_cast<uint16_t>(entries.size()));
    putLE<uint16_t>(out, static_cast<uint16_t>(entries.size()));
    putLE<uint32_t>(out, cdSize);
    putLE<uint32_t>(out, cdStart);
    putLE<uint16_t>(out, 0); // comment length

    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Could not open '" + path + "' for writing");
    f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
}

struct CentralDirEntry {
    std::string name;
    uint16_t method;
    uint32_t compressedSize;
    uint32_t uncompressedSize;
    uint32_t localHeaderOffset;
};

std::vector<uint8_t> readWholeFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Could not open '" + path + "' for reading");
    in.seekg(0, std::ios::end);
    const auto size = in.tellg();
    if (size < 0) throw std::runtime_error("Could not read '" + path + "'");
    std::vector<uint8_t> buf(static_cast<size_t>(size));
    in.seekg(0);
    if (!buf.empty()) in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
    return buf;
}

std::vector<CentralDirEntry> parseCentralDirectory(const std::vector<uint8_t>& file, const std::string& path) {
    // End Of Central Directory record is at least 22 bytes; scan backward
    // for its signature (no ZIP comment expected in a 3MF we care about, but
    // scan a reasonable trailing window regardless of comment length).
    if (file.size() < 22) throw std::runtime_error("'" + path + "' is not a valid ZIP archive");
    const size_t maxBack = std::min<size_t>(file.size(), 22 + 65536);
    size_t eocd = std::string::npos;
    for (size_t back = 22; back <= maxBack; ++back) {
        const size_t pos = file.size() - back;
        if (readU32(&file[pos]) == 0x06054b50u) {
            eocd = pos;
            break;
        }
    }
    if (eocd == std::string::npos) throw std::runtime_error("'" + path + "' is not a valid ZIP archive (no EOCD)");

    const uint16_t entryCount = readU16(&file[eocd + 10]);
    const uint32_t cdOffset = readU32(&file[eocd + 16]);
    if (cdOffset >= file.size()) throw std::runtime_error("'" + path + "': corrupt central directory offset");

    std::vector<CentralDirEntry> entries;
    size_t p = cdOffset;
    for (uint16_t i = 0; i < entryCount; ++i) {
        if (p + 46 > file.size() || readU32(&file[p]) != 0x02014b50u) {
            throw std::runtime_error("'" + path + "': corrupt central directory entry");
        }
        const uint16_t method = readU16(&file[p + 10]);
        const uint32_t compressedSize = readU32(&file[p + 20]);
        const uint32_t uncompressedSize = readU32(&file[p + 24]);
        const uint16_t nameLen = readU16(&file[p + 28]);
        const uint16_t extraLen = readU16(&file[p + 30]);
        const uint16_t commentLen = readU16(&file[p + 32]);
        const uint32_t localHeaderOffset = readU32(&file[p + 42]);
        if (p + 46 + nameLen > file.size()) throw std::runtime_error("'" + path + "': corrupt central directory entry name");
        std::string name(reinterpret_cast<const char*>(&file[p + 46]), nameLen);
        entries.push_back(CentralDirEntry{std::move(name), method, compressedSize, uncompressedSize, localHeaderOffset});
        p += 46 + nameLen + extraLen + commentLen;
    }
    return entries;
}

std::vector<uint8_t> extractEntry(const std::vector<uint8_t>& file, const std::string& path, const CentralDirEntry& entry) {
    if (entry.method != 0 && entry.method != 8) {
        throw std::runtime_error("'" + path + "': entry '" + entry.name +
                                  "' uses an unsupported ZIP compression method (" + std::to_string(entry.method) +
                                  "; only stored and DEFLATE are supported)");
    }
    const size_t lh = entry.localHeaderOffset;
    if (lh + 30 > file.size() || readU32(&file[lh]) != 0x04034b50u) {
        throw std::runtime_error("'" + path + "': corrupt local file header for '" + entry.name + "'");
    }
    const uint16_t nameLen = readU16(&file[lh + 26]);
    const uint16_t extraLen = readU16(&file[lh + 28]);
    const size_t dataStart = lh + 30 + nameLen + extraLen;
    if (dataStart + entry.compressedSize > file.size()) {
        throw std::runtime_error("'" + path + "': truncated entry '" + entry.name + "'");
    }
    if (entry.method == 0) {
        return std::vector<uint8_t>(file.begin() + static_cast<std::ptrdiff_t>(dataStart),
                                     file.begin() + static_cast<std::ptrdiff_t>(dataStart + entry.compressedSize));
    }

    // method == 8 (DEFLATE): ZIP's compressed stream is exactly a raw
    // DEFLATE stream (no zlib/gzip wrapper), so stb_image's
    // "_noheader_" decoder -- built for that same raw form when
    // decompressing PNG IDAT data -- applies directly.
    std::vector<uint8_t> out(entry.uncompressedSize);
    const int decoded = stbi_zlib_decode_noheader_buffer(
        reinterpret_cast<char*>(out.data()), static_cast<int>(out.size()),
        reinterpret_cast<const char*>(&file[dataStart]), static_cast<int>(entry.compressedSize));
    if (decoded < 0 || static_cast<size_t>(decoded) != entry.uncompressedSize) {
        throw std::runtime_error("'" + path + "': failed to inflate DEFLATE-compressed entry '" + entry.name + "'");
    }
    return out;
}

} // namespace

void writeStoredZip(const std::string& path, const std::vector<ZipEntry>& entries) {
    std::vector<PlacedEntry> placed;
    placed.reserve(entries.size());
    for (const auto& e : entries) {
        placed.push_back(PlacedEntry{e.name, 0, e.data, crc32(e.data.data(), e.data.size()),
                                      static_cast<uint32_t>(e.data.size())});
    }
    writeZipEntries(path, placed);
}

void writeDeflateZip(const std::string& path, const std::vector<ZipEntry>& entries) {
    std::vector<PlacedEntry> placed;
    placed.reserve(entries.size());
    for (const auto& e : entries) {
        const uint32_t crc = crc32(e.data.data(), e.data.size());
        std::vector<uint8_t> compressed = deflateRaw(e.data);
        // A tiny or already-incompressible entry can end up no smaller (or
        // even larger, once framing is counted) once compressed -- store it
        // raw instead in that case, same as deflateRaw() returning empty on
        // outright failure.
        if (!compressed.empty() && compressed.size() < e.data.size()) {
            placed.push_back(PlacedEntry{e.name, 8, std::move(compressed), crc, static_cast<uint32_t>(e.data.size())});
        } else {
            placed.push_back(PlacedEntry{e.name, 0, e.data, crc, static_cast<uint32_t>(e.data.size())});
        }
    }
    writeZipEntries(path, placed);
}

std::vector<uint8_t> readStoredZipEntry(const std::string& path, const std::string& name) {
    const std::vector<uint8_t> file = readWholeFile(path);
    const std::vector<CentralDirEntry> entries = parseCentralDirectory(file, path);
    for (const auto& e : entries) {
        if (e.name == name) return extractEntry(file, path, e);
    }
    throw std::runtime_error("'" + path + "': no entry named '" + name + "'");
}

std::vector<uint8_t> readStoredZipEntryBySuffix(const std::string& path, const std::string& suffix) {
    const std::vector<uint8_t> file = readWholeFile(path);
    const std::vector<CentralDirEntry> entries = parseCentralDirectory(file, path);
    const auto lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
        return s;
    };
    const std::string lowerSuffix = lower(suffix);
    for (const auto& e : entries) {
        const std::string lowerName = lower(e.name);
        if (lowerName.size() >= lowerSuffix.size() &&
            lowerName.compare(lowerName.size() - lowerSuffix.size(), lowerSuffix.size(), lowerSuffix) == 0) {
            return extractEntry(file, path, e);
        }
    }
    throw std::runtime_error("'" + path + "': no entry ending with '" + suffix + "'");
}

} // namespace oscadeval
