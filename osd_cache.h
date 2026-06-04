#ifndef OSD_CACHE_H
#define OSD_CACHE_H

#include <RgaApi.h>
#include <im2d.h>

#ifdef __cplusplus
extern "C" {
#endif

// ����һ����ӡ�¡�
typedef struct {
    unsigned char* rgba_data; // ������ص������ڴ��ַ
    rga_buffer_t rga_buf;     // RGA �ܿ����� Buffer �ṹ
    int width;                // ���ӡ�µĿ���
    int height;               // ���ӡ�µĸ߶�
} GlyphStamp;

// ȫ��ӡ�²ֿ�
extern GlyphStamp g_stamp_labels[6]; // �� 5 �������ǩ
extern GlyphStamp g_stamp_digits[10]; // �� '0' �� '9'
extern GlyphStamp g_stamp_percent;    // �� '%'
extern GlyphStamp g_stamp_score_digits[6][10];
extern GlyphStamp g_stamp_score_percent[6];
extern GlyphStamp g_stamp_score_text[6][100];

unsigned int osd_class_color_rgb(int class_id);

// ��ʼ��������������һ�Σ�
int osd_cache_init(const char* font_path, int font_size);
// ���ٺ������ػ���һ�Σ�
void osd_cache_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // OSD_CACHE_H
