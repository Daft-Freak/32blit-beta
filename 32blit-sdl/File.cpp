#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

#include <string>

#include "SDL.h"

#include "File.hpp"

static std::string basePath;

void setup_base_path()
{
  auto basePathPtr = SDL_GetBasePath();
  basePath = std::string(basePathPtr);
  SDL_free(basePathPtr);
}

void *open_file(std::string name, int mode) {
  const char *str_mode;

  if(mode == blit::OpenMode::read)
    str_mode = "rb";
  else if(mode == blit::OpenMode::write)
    str_mode = "wb";
  else if(mode == (blit::OpenMode::read | blit::OpenMode::write))
    str_mode = "r+";
  else
    return nullptr;

  auto file = SDL_RWFromFile((basePath + name).c_str(), str_mode);
  return file;
}

int32_t read_file(void *fh, uint32_t offset, uint32_t length, char *buffer) {
  auto file = (SDL_RWops *)fh;

  if(file && SDL_RWseek(file, offset, RW_SEEK_SET) != -1) {
    size_t bytes_read = SDL_RWread(file, buffer, 1, length);

    if(bytes_read > 0)
      return bytes_read;
  }

  return -1;
}

int32_t write_file(void *fh, uint32_t offset, uint32_t length, const char *buffer) {
  auto file = (SDL_RWops *)fh;

  if(file && SDL_RWseek(file, offset, RW_SEEK_SET) != -1) {
    size_t bytes_read = SDL_RWwrite(file, buffer, 1, length);

    if(bytes_read > 0)
      return bytes_read;
  }

  return -1;
}

int32_t close_file(void *fh) {
  return SDL_RWclose((SDL_RWops *)fh) == 0 ? 0 : -1;
}

uint32_t get_file_length(void *fh)
{
  auto file = (SDL_RWops *)fh;
  SDL_RWseek(file, 0, RW_SEEK_END);

  return SDL_RWtell(file);
}

std::vector<blit::FileInfo> list_files(std::string path) {
  std::vector<blit::FileInfo> ret;

  std::error_code err;
  for(auto &entry: fs::directory_iterator(basePath + path, fs::directory_options::follow_directory_symlink, err)) {
    blit::FileInfo info;

    info.name = entry.path().filename().generic_string();

    info.flags = 0;

    if(entry.status().type() == fs::file_type::directory)
      info.flags |= blit::FileFlags::directory;

    ret.push_back(info);
  }

  return ret;
}

bool file_exists(std::string path) {
  std::error_code err;
  return fs::status(path, err).type() == fs::file_type::regular;
}

bool directory_exists(std::string path) {
  std::error_code err;
  return fs::status(path, err).type() == fs::file_type::directory;
}

bool create_directory(std::string path) {
  std::error_code err;
  return directory_exists(path) || fs::create_directory(path, err);
}
