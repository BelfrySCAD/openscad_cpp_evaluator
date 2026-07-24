#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace oscadeval {

// A single stored (uncompressed) ZIP entry.
struct ZipEntry {
    std::string name;
    std::vector<uint8_t> data;
};

// Writes a ZIP archive containing exactly `entries`, all with STORED
// (uncompressed) compression -- fully valid per the ZIP spec (every real
// reader, including Windows Explorer/7-Zip/real OpenSCAD, accepts stored
// entries), just larger than a DEFLATE-compressed archive would be. Throws
// std::runtime_error if the file can't be opened for writing.
void writeStoredZip(const std::string& path, const std::vector<ZipEntry>& entries);

// Same, but compresses each entry with DEFLATE (method 8) via
// stb_image_write.h's own zlib compressor (already vendored for
// surface()'s PNG writing needs) -- meaningfully smaller output for
// text-heavy entries like 3MF's XML. An entry that doesn't actually shrink
// under compression (tiny or already-incompressible data) is stored
// uncompressed instead, same as any real ZIP writer would. Throws
// std::runtime_error if the file can't be opened for writing.
void writeDeflateZip(const std::string& path, const std::vector<ZipEntry>& entries);

// Reads one entry (by exact name match) from a ZIP archive via its central
// directory (authoritative regardless of how local headers were written).
// Supports both STORED (method 0, this project's own writeStoredZip()
// output) and DEFLATE (method 8, what most third-party tools -- slicers,
// other CAD software -- actually write) entries; DEFLATE decoding reuses
// stb_image's own raw-inflate decoder (already a project dependency for
// surface()'s PNG loader). Any other compression method throws
// std::runtime_error naming the entry and method.
std::vector<uint8_t> readStoredZipEntry(const std::string& path, const std::string& name);

// Same, but matches the first entry whose name ends with `suffix`
// (case-insensitive) -- used to find "*/3dmodel.model" inside a 3MF archive
// without needing to know its exact containing-folder name.
std::vector<uint8_t> readStoredZipEntryBySuffix(const std::string& path, const std::string& suffix);

} // namespace oscadeval
