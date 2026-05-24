#ifndef OSD_CACHE_H
#define OSD_CACHE_H

#include <RgaApi.h>
#include <im2d.h>

#ifdef __cplusplus
extern "C" {
#endif

// 定义一个“印章”
typedef struct {
    unsigned char* rgba_data; // 存放像素的虚拟内存地址
    rga_buffer_t rga_buf;     // RGA 能看懂的 Buffer 结构
    int width;                // 这个印章的宽度
    int height;               // 这个印章的高度
} GlyphStamp;

// 全局印章仓库
extern GlyphStamp g_stamp_labels[5]; // 存 5 个分类标签
extern GlyphStamp g_stamp_digits[10]; // 存 '0' 到 '9'
extern GlyphStamp g_stamp_percent;    // 存 '%'

// 初始化函数（开机调一次）
int osd_cache_init(const char* font_path, int font_size);
// 销毁函数（关机调一次）
void osd_cache_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // OSD_CACHE_H