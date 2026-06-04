#include "osd_cache.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // ���������ͷ�ļ�

#include <ft2build.h>
#include FT_FREETYPE_H

const char* project_labels[6] = {
    "helmet", "no-helmet", "no-vest", "person", "vest", "fire"
};

GlyphStamp g_stamp_labels[6];
GlyphStamp g_stamp_digits[10];
GlyphStamp g_stamp_percent;
GlyphStamp g_stamp_score_digits[6][10];
GlyphStamp g_stamp_score_percent[6];
GlyphStamp g_stamp_score_text[6][100];

static FT_Library ft_lib = NULL;
static FT_Face ft_face = NULL;

static const unsigned int g_class_colors[6] = {
    0x2ecc71, // helmet: green
    0xff2d55, // no-helmet: red
    0xff9500, // no-vest: orange
    0x007aff, // person: blue
    0x00a6d6, // vest: blue cyan
    0xaf52de  // fire: purple
};

unsigned int osd_class_color_rgb(int class_id) {
    if (class_id >= 0 && class_id < 6) {
        return g_class_colors[class_id];
    }
    return 0xffffff;
}

static int create_stamp_colored(const char* text, GlyphStamp* stamp, unsigned int color) {
    if (!ft_face) return -1;

    // 1. �����ı�����
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
    
    stamp->width = (total_width == 0) ? 12 : total_width; // ǿ������ռλ
    stamp->height = (max_bearing_y + min_descent == 0) ? 20 : (max_bearing_y + min_descent);
    
    // �������޸�������������� Stride (������ 4 �ı���)
    int stride = (stamp->width + 3) & ~3; 
    int size = stride * stamp->height * 4;

    // 2. ���� 64 �ֽڶ�����ڴ�
    if (posix_memalign((void**)&stamp->rgba_data, 64, size) != 0) {
        return -1;
    }
    memset(stamp->rgba_data, 0, size);

    // 3. �������
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
                    if (alpha >= 96) {
                        // �������޸���������ʹ�� stride ���������������� width
                        int index = (img_y * stride + img_x) * 4;
                        stamp->rgba_data[index + 0] = (color >> 16) & 0xff; // R
                        stamp->rgba_data[index + 1] = (color >> 8) & 0xff;  // G
                        stamp->rgba_data[index + 2] = color & 0xff;         // B
                        stamp->rgba_data[index + 3] = 255; // Alpha
                    }
                }
            }
        }
        pen_x += (ft_face->glyph->advance.x >> 6);
    }

    // 4. ��װ RGA Buffer ���ֶ����� Stride
    stamp->rga_buf = wrapbuffer_virtualaddr(
        (void*)stamp->rgba_data, 
        stamp->width, 
        stamp->height, 
        RK_FORMAT_RGBA_8888
    );
    // ���ؼ�������ʽָ�� wstride
    stamp->rga_buf.wstride = stride;
    
    return 0;
}

static int create_stamp(const char* text, GlyphStamp* stamp) {
    return create_stamp_colored(text, stamp, 0x00ff00);
}

int osd_cache_init(const char* font_path, int font_size) {
    if (FT_Init_FreeType(&ft_lib)) return -1;
    if (FT_New_Face(ft_lib, font_path, 0, &ft_face)) return -1;
    FT_Set_Pixel_Sizes(ft_face, 0, font_size);

    for (int i = 0; i < 6; i++) create_stamp_colored(project_labels[i], &g_stamp_labels[i], 0xffffff);
    
    char digit_str[2] = {0};
    for (int i = 0; i < 10; i++) {
        digit_str[0] = '0' + i;
        create_stamp(digit_str, &g_stamp_digits[i]);
    }
    create_stamp("%", &g_stamp_percent);

    for (int cls = 0; cls < 6; cls++) {
        unsigned int color = 0xffffff;
        for (int i = 0; i < 10; i++) {
            digit_str[0] = '0' + i;
            create_stamp_colored(digit_str, &g_stamp_score_digits[cls][i], color);
        }
        create_stamp_colored("%", &g_stamp_score_percent[cls], color);

        char score_str[4];
        for (int score = 0; score < 100; score++) {
            snprintf(score_str, sizeof(score_str), "%02d%%", score);
            create_stamp_colored(score_str, &g_stamp_score_text[cls][score], color);
        }
    }

    return 0;
}

void osd_cache_deinit(void) {
    for (int i = 0; i < 6; i++) { if(g_stamp_labels[i].rgba_data) free(g_stamp_labels[i].rgba_data); }
    for (int i = 0; i < 10; i++) { if(g_stamp_digits[i].rgba_data) free(g_stamp_digits[i].rgba_data); }
    if(g_stamp_percent.rgba_data) free(g_stamp_percent.rgba_data);
    for (int cls = 0; cls < 6; cls++) {
        for (int i = 0; i < 10; i++) {
            if (g_stamp_score_digits[cls][i].rgba_data) free(g_stamp_score_digits[cls][i].rgba_data);
        }
        if (g_stamp_score_percent[cls].rgba_data) free(g_stamp_score_percent[cls].rgba_data);
        for (int score = 0; score < 100; score++) {
            if (g_stamp_score_text[cls][score].rgba_data) free(g_stamp_score_text[cls][score].rgba_data);
        }
    }
    FT_Done_Face(ft_face);
    FT_Done_FreeType(ft_lib);
}
