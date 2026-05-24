#include "osd_cache.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // 必须包含此头文件

#include <ft2build.h>
#include FT_FREETYPE_H

const char* project_labels[5] = {
    "kask var", "kask yok", "yelek var", "yelek yok", "fire"
};

GlyphStamp g_stamp_labels[5];
GlyphStamp g_stamp_digits[10];
GlyphStamp g_stamp_percent;

static FT_Library ft_lib = NULL;
static FT_Face ft_face = NULL;

static int create_stamp(const char* text, GlyphStamp* stamp) {
    if (!ft_face) return -1;

    // 1. 计算文本宽高
    int total_width = 0;
    int max_bearing_y = 0;
    int min_descent = 0;
    int len = strlen(text);
    
    for (int i = 0; i < len; i++) {
        if (FT_Load_Char(ft_face, text[i], FT_LOAD_RENDER)) continue;
        total_width += (ft_face->glyph->advance.x >> 6);
        int bearing_y = ft_face->glyph->bitmap_top;
        int descent = ft_face->glyph->bitmap.rows - bearing_y;
        if (bearing_y > max_bearing_y) max_bearing_y = bearing_y;
        if (descent > min_descent) min_descent = descent;
    }
    
    stamp->width = (total_width == 0) ? 12 : total_width; // 强制至少占位
    stamp->height = (max_bearing_y + min_descent == 0) ? 20 : (max_bearing_y + min_descent);
    
    // 【核心修复】：计算对齐后的 Stride (必须是 4 的倍数)
    int stride = (stamp->width + 3) & ~3; 
    int size = stride * stamp->height * 4;

    // 2. 申请 64 字节对齐的内存
    if (posix_memalign((void**)&stamp->rgba_data, 64, size) != 0) {
        return -1;
    }
    memset(stamp->rgba_data, 0, size);

    // 3. 填充像素
    int pen_x = 0;
    for (int i = 0; i < len; i++) {
        if (FT_Load_Char(ft_face, text[i], FT_LOAD_RENDER)) continue;
        
        FT_Bitmap* bitmap = &ft_face->glyph->bitmap;
        int y_offset = max_bearing_y - ft_face->glyph->bitmap_top;
        int x_offset = pen_x + ft_face->glyph->bitmap_left;

        for (unsigned int r = 0; r < bitmap->rows; r++) {
            for (unsigned int c = 0; c < bitmap->width; c++) {
                int img_y = y_offset + r;
                int img_x = x_offset + c;
                if (img_x >= 0 && img_x < stamp->width && img_y >= 0 && img_y < stamp->height) {
                    unsigned char alpha = bitmap->buffer[r * bitmap->pitch + c];
                    if (alpha > 0) {
                        // 【核心修复】：必须使用 stride 计算索引，而不是 width
                        int index = (img_y * stride + img_x) * 4;
                        stamp->rgba_data[index + 0] = 0;   // R
                        stamp->rgba_data[index + 1] = 255; // G
                        stamp->rgba_data[index + 2] = 0;   // B
                        stamp->rgba_data[index + 3] = alpha; // Alpha
                    }
                }
            }
        }
        pen_x += (ft_face->glyph->advance.x >> 6);
    }

    // 4. 包装 RGA Buffer 并手动设置 Stride
    stamp->rga_buf = wrapbuffer_virtualaddr(
        (void*)stamp->rgba_data, 
        stamp->width, 
        stamp->height, 
        RK_FORMAT_RGBA_8888
    );
    // 【关键】：显式指定 wstride
    stamp->rga_buf.wstride = stride;
    
    return 0;
}

int osd_cache_init(const char* font_path, int font_size) {
    if (FT_Init_FreeType(&ft_lib)) return -1;
    if (FT_New_Face(ft_lib, font_path, 0, &ft_face)) return -1;
    FT_Set_Pixel_Sizes(ft_face, 0, font_size);

    for (int i = 0; i < 5; i++) create_stamp(project_labels[i], &g_stamp_labels[i]);
    
    char digit_str[2] = {0};
    for (int i = 0; i < 10; i++) {
        digit_str[0] = '0' + i;
        create_stamp(digit_str, &g_stamp_digits[i]);
    }
    create_stamp("%", &g_stamp_percent);
    return 0;
}

void osd_cache_deinit(void) {
    for (int i = 0; i < 5; i++) { if(g_stamp_labels[i].rgba_data) free(g_stamp_labels[i].rgba_data); }
    for (int i = 0; i < 10; i++) { if(g_stamp_digits[i].rgba_data) free(g_stamp_digits[i].rgba_data); }
    if(g_stamp_percent.rgba_data) free(g_stamp_percent.rgba_data);
    FT_Done_Face(ft_face);
    FT_Done_FreeType(ft_lib);
}