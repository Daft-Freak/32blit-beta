#pragma once

#include <cstdint>

#include "surface.hpp"
#include "../engine/engine.hpp"
#include "../types/size.hpp"
#include "../types/point.hpp"
#include "../types/mat3.hpp"

namespace blit {
  enum MapLoadFlags {
    COPY_TILES       = (1 << 0), /// copy tile data
    COPY_TRANSFORMS  = (1 << 1), /// copy per-tile transform data
    LAYER_TRANSFORMS = (1 << 2), /// each layer can be transformed (optionally per-line), otherwise only scrolling is supported
    TILES_16BIT      = (1 << 3), /// internal flag set if tile data is 16-bit
  };

  enum TMXFlags {
    TMX_16BIT      = (1 << 0), /// tile data is 16-bit
    TMX_TRANSFORMS = (1 << 1), /// transform data is included after the tile data
    TMX_NAMES      = (1 << 2), /// null-terminated layer names are included after the tile/transform data
  };

  /// struct header generated by the `output_struct` option
  #pragma pack(push,1)
  struct TMX {
      char head[4];
      uint16_t header_length;
      uint16_t flags;
      uint16_t empty_tile;
      uint16_t width;
      uint16_t height;
      uint16_t layers;
      uint8_t data[];
  };
  #pragma pack(pop)

  // A `tilemap` describes a grid of tiles with optional transforms
  struct TileLayer {
    Size bounds;

    uint8_t *tiles;
    uint8_t *transforms;
    Surface *sprites;
    Mat3 transform = Mat3::identity();

    enum {
      NONE = 0,           // draw nothing
      REPEAT = 1,         // infinite repeat
      DEFAULT_FILL = 2,   // fill with default tile
      CLAMP_TO_EDGE = 3,  // repeats the tile at the edge
    } repeat_mode = NONE; // determines what to do when drawing outside of the layer bounds.
    uint16_t default_tile_id;

    int empty_tile_id = -1;

    int load_flags = 0;

    std::string name;

    TileLayer(uint8_t *tiles, uint8_t *transforms, Size bounds, Surface *sprites);
    virtual ~TileLayer();

    virtual void draw(Surface *dest, Rect viewport) = 0;

    inline int32_t offset(const Point &p) {return offset(p.x, p.y);}
    int32_t offset(int16_t x, int16_t y);
    uint16_t tile_at(const Point &p);
    uint8_t transform_at(const Point &p);

    void set_tiles(uint8_t *tiles, bool copy = true);
  };

  struct SimpleTileLayer : public TileLayer {
    SimpleTileLayer(uint8_t *tiles, uint8_t *transforms, Size bounds, Surface *sprites);

    static SimpleTileLayer *load_tmx(const uint8_t *asset, Surface *sprites, int layer = 0, int flags = COPY_TILES | COPY_TRANSFORMS);
    static SimpleTileLayer *load_tmx(File &file, Surface *sprites, int layer = 0, int flags = COPY_TILES | COPY_TRANSFORMS);

    void draw(Surface *dest, Rect viewport) override;
  };

  struct TransformedTileLayer : public TileLayer {
    TransformedTileLayer(uint8_t *tiles, uint8_t *transforms, Size bounds, Surface *sprites);

    static TransformedTileLayer *load_tmx(const uint8_t *asset, Surface *sprites, int layer = 0, int flags = COPY_TILES | COPY_TRANSFORMS);
    static TransformedTileLayer *load_tmx(File &file, Surface *sprites, int layer = 0, int flags = COPY_TILES | COPY_TRANSFORMS);

    void draw(Surface *dest, Rect viewport) override {draw(dest, viewport, nullptr);}
    void draw(Surface *dest, Rect viewport, std::function<Mat3(uint8_t)> scanline_callback);

    void mipmap_texture_span(Surface *dest, Point s, uint16_t c, Vec2 swc, Vec2 ewc);
    void texture_span(Surface *dest, Point s, unsigned int c, Vec2 swc, Vec2 ewc, Surface *src = nullptr, unsigned int mipmap_index = 0);

  private:
    template<class index_type>
    void do_texture_span(Surface *dest, Point s, unsigned int c, Vec2 swc, Vec2 ewc, Surface *src, unsigned int mipmap_index);

    int32_t fast_offset(int16_t x, int16_t y);
  };

  /// Multi-layered tile map
  class TiledMap {
  public:
    TiledMap(Size bounds, unsigned num_layers, Surface *sprites, int flags = 0);
    TiledMap(const uint8_t *asset, Surface *sprites, int flags = COPY_TILES | COPY_TRANSFORMS);
    TiledMap(const std::string &filename, Surface *sprites, int flags = COPY_TILES | COPY_TRANSFORMS);

    virtual ~TiledMap();

    /// Draw map to `screen`
    void draw() {draw(&screen, Rect({0, 0}, screen.bounds));}

    void draw(Surface *dest, Rect viewport);

    void draw(std::function<Mat3(uint8_t)> scanline_callback) {draw(&screen, Rect({0, 0}, screen.bounds), scanline_callback);}
    void draw(Surface *dest, Rect viewport, std::function<Mat3(uint8_t)> scanline_callback);

    unsigned get_num_layers() const {return num_layers;}

    TileLayer *get_layer(unsigned index);
    TileLayer *get_layer(std::string_view name);

    unsigned get_layer_index(std::string_view name) const;

    Size get_bounds() const;

    void set_scroll_position(Point scroll_position);
    void set_scroll_position(unsigned layer, Point scroll_position);

    void set_transform(Mat3 transform);
    void set_transform(unsigned layer, Mat3 transform);

    // flags
    void add_flags(unsigned layer_index, uint16_t tile_id, uint8_t new_flags);
    void add_flags(unsigned layer_index, std::initializer_list<uint16_t> tile_ids, uint8_t new_flags);

    uint8_t get_flags(Point p) const;
    bool has_flag(Point p, uint8_t flag) const;

  private:
    unsigned num_layers = 0;
    TileLayer **layers = nullptr;

    uint8_t *flags = nullptr;
  };

  using TileMap [[deprecated("Use TransformedTileLayer (or TiledMap)")]] = TransformedTileLayer;
}
