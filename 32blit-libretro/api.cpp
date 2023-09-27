#ifdef WIN32
#include <direct.h>
#include <shlobj.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#endif

#include <random>

#include "api.hpp"
#include "consts.hpp"

using namespace blit;

static blit::AudioChannel channels[CHANNEL_COUNT];

static auto default_screen_format = blit::PixelFormat::RGB;
static SurfaceInfo cur_surf_info;
static constexpr Size hires_screen_size(screen_width, screen_height);
static constexpr Size lores_screen_size(screen_width / 2, screen_height / 2);

static uint8_t screen_fb[screen_width * screen_height * 4];

static std::string base_path, save_path;

static bool set_screen_mode_format(ScreenMode new_mode, SurfaceTemplate &new_surf_template) {
  new_surf_template.data = screen_fb;

  if(new_surf_template.format == (PixelFormat)-1)
    new_surf_template.format = default_screen_format;

  switch(new_mode) {
    case ScreenMode::lores:
      new_surf_template.bounds = lores_screen_size;
      break;
    case ScreenMode::hires:
    case ScreenMode::hires_palette:
      new_surf_template.bounds = hires_screen_size;
      break;
  }

  if(!screen_mode_changed(new_surf_template))
    return false;

  return true;
}

void set_screen_palette(const Pen *colours, int num_cols) {

}

// blit random callback
#ifdef __MINGW32__
// Windows/MinGW doesn't support a non-deterministic source of randomness, so we fall back upon the age-old time seed once more
// Without this, random_device() will always return the same number and thus our mersenne twister will always produce the same sequence.
static std::mt19937 random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
#else
static std::random_device random_device;
static std::mt19937 random_generator(random_device());
#endif
static std::uniform_int_distribution<uint32_t> random_distribution;
static uint32_t blit_random() {
  return random_distribution(random_generator);
}

// files
static std::string map_path(const std::string &path) {
  // check if the path is under the save path
  if(path.compare(0, save_path.length(), save_path) == 0)
    return path;

  // otherwise it should be under the base path
  return base_path + path;
}
#include <cstring>
static void *open_file(const std::string &name, int mode) {
  const char *str_mode;

  if(mode == blit::OpenMode::read)
    str_mode = "rb";
  else if(mode == blit::OpenMode::write)
    str_mode = "wb";
  else if(mode == (blit::OpenMode::read | blit::OpenMode::write))
    str_mode = "r+b";
  else
    return nullptr;

  auto file = fopen(map_path(name).c_str(), str_mode);

  return file;
}

static int32_t read_file(void *fh, uint32_t offset, uint32_t length, char *buffer) {
  FILE *f = (FILE *)fh;

  int r = fseek(f, offset, SEEK_SET);

  if(r == 0){
    r = fread(buffer, 1, length, f);

    if(!ferror(f))
      return r;
  }

  return -1;
}

static int32_t write_file(void *fh, uint32_t offset, uint32_t length, const char *buffer) {
  FILE *f = (FILE *)fh;

  int r = fseek(f, offset, SEEK_SET);

  if(r == 0){
    r = fwrite(buffer, 1, length, f);

    if(!ferror(f))
      return r;
  }

  return -1;
}

static int32_t close_file(void *fh) {
  return fclose((FILE *)fh);
}

static uint32_t get_file_length(void *fh) {
  FILE *f = (FILE *)fh;

  if(fseek(f, 0, SEEK_END) < 0)
    return 0;

  return ftell(f);
}

static void list_files(const std::string &path, std::function<void(blit::FileInfo &)> callback) {
#ifdef WIN32
  HANDLE file;
  WIN32_FIND_DATAA findData;
  file = FindFirstFileA((map_path(path) + "\\*").c_str(), &findData);

  if(file == INVALID_HANDLE_VALUE)
    return;

  do
  {
    blit::FileInfo info;
    info.name = findData.cFileName;

    if(info.name == "." || info.name == "..")
      continue;

    info.flags = 0;
    info.size = findData.nFileSizeLow;

    if(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      info.flags |= blit::FileFlags::directory;

    callback(info);
  }
  while(FindNextFileA(file, &findData) != 0);

  FindClose(file);

#else
  auto mapped_path = map_path(path);
  auto dir = opendir(mapped_path.c_str());

  if(!dir)
    return;

  struct dirent *ent;

  while((ent = readdir(dir))) {
    blit::FileInfo info;

    info.name = ent->d_name;

    if(info.name == "." || info.name == "..")
      continue;

    struct stat stat_buf;

    if(stat((mapped_path + "/" + info.name).c_str(), &stat_buf) < 0)
      continue;

    info.flags = 0;
    info.size = stat_buf.st_size;

    if(S_ISDIR(stat_buf.st_mode))
      info.flags |= blit::FileFlags::directory;

    callback(info);
  }

  closedir(dir);
#endif
}

static bool file_exists(const std::string &path) {
#ifdef WIN32
	DWORD attribs = GetFileAttributesA(map_path(path).c_str());
	return (attribs != INVALID_FILE_ATTRIBUTES && !(attribs & FILE_ATTRIBUTE_DIRECTORY));
#else
  struct stat stat_buf;
  return (stat(map_path(path).c_str(), &stat_buf) == 0 && S_ISREG(stat_buf.st_mode));
#endif
}

static bool directory_exists(const std::string &path) {
#ifdef WIN32
	DWORD attribs = GetFileAttributesA(map_path(path).c_str());
	return (attribs != INVALID_FILE_ATTRIBUTES && (attribs & FILE_ATTRIBUTE_DIRECTORY));
#else
  struct stat stat_buf;
  return (stat(map_path(path).c_str(), &stat_buf) == 0 && S_ISDIR(stat_buf.st_mode));
#endif
}

static bool create_directory(const std::string &path) {
#ifdef WIN32
  return _mkdir(map_path(path).c_str()) == 0 || errno == EEXIST;
#else
  return mkdir(map_path(path).c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

static bool rename_file(const std::string &old_name, const std::string &new_name) {
  return rename(map_path(old_name).c_str(), map_path(new_name).c_str()) == 0;
}

static bool remove_file(const std::string &path) {
  return remove(map_path(path).c_str()) == 0;
}

static const char *get_save_path() {
  return save_path.c_str();
}

static bool is_storage_available() {
  return true;
}

// profiler
static uint32_t get_max_us_timer() {
  return 0xFFFFFFFF;
}

void blit_api_init(const char *file_base_dir, const char *save_dir) {
  base_path = std::string(file_base_dir) + "/";
  save_path = std::string(save_dir) + "/" + metadata_title + "/";

  if(!::directory_exists(save_path))
    ::create_directory(save_path);

  api.channels = ::channels;

  api.set_screen_mode = nullptr;
  api.set_screen_palette = ::set_screen_palette;
  api.set_screen_mode_format = ::set_screen_mode_format;
  api.now = ::get_current_time;
  api.random = ::blit_random;
  // api.exit = ::exit;

  // serial debug
  api.debug = ::debug_message;

  // files
  api.open_file = ::open_file;
  api.read_file = ::read_file;
  api.write_file = ::write_file;
  api.close_file = ::close_file;
  api.get_file_length = ::get_file_length;
  api.list_files = ::list_files;
  api.file_exists = ::file_exists;
  api.directory_exists = ::directory_exists;
  api.create_directory = ::create_directory;
  api.rename_file = ::rename_file;
  api.remove_file = ::remove_file;
  api.get_save_path = ::get_save_path;
  api.is_storage_available = ::is_storage_available;

  // profiler
  // api.enable_us_timer = ::enable_us_timer;
  api.get_us_timer = ::get_us_timer;
  api.get_max_us_timer = ::get_max_us_timer;

  // jpeg
  // api.decode_jpeg_buffer = ::decode_jpeg_buffer;
  // api.decode_jpeg_file = ::decode_jpeg_file;

  // launcher
  // api.launch = ::launch;
  // api.erase_game = ::erase_game;
  // api.get_type_handler_metadata = ::get_type_handler_metadata;

  // api.get_launch_path = ::get_launch_path;

  // multiplayer
  // api.is_multiplayer_connected = ::is_multiplayer_connected;
  // api.set_multiplayer_enabled = ::set_multiplayer_enabled;
  // api.send_message = ::send_multiplayer_message;

  // api.flash_to_tmp = ::flash_to_tmp;
  // api.tmp_file_closed = ::tmp_file_closed;

  // api.get_metadata = ::get_metadata;
}
