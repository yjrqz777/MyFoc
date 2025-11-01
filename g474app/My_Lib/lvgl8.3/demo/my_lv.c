#include "my_lv.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "tim.h"
#include "gpio.h"
#include "stdio.h"
void lvgl_tick(void const *argument)
{
    /* USER CODE BEGIN lvgl_tick */
    /* Infinite loop */
	
   lv_tick_inc(1);
    /* USER CODE END lvgl_tick */
}



static void slider_event_cb(lv_event_t *e);
static lv_obj_t *slider_label;

/**
 * A default slider with a label displaying the current value
 */
void lv_example_slider_1(void)
{
    /*Create a slider in the center of the display*/
    lv_obj_t *slider = lv_slider_create(lv_scr_act());
    lv_obj_center(slider);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /*Create a label below the slider*/
    slider_label = lv_label_create(lv_scr_act());
    lv_label_set_text(slider_label, "0%");

    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

static void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    uint16_t value = (int)lv_slider_get_value(slider) * 10;
    char buf[8];
    uint16_t blk = 300;
    lv_snprintf(buf, sizeof(buf), "%d%%", value / 10);
    blk = value < 300 ? 300 : value;

    htim2.Instance->CCR1 = blk;
    lv_label_set_text(slider_label, buf);
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

#define CANVAS_WIDTH 100
#define CANVAS_HEIGHT 150

void lv_example_canvas_2(void)
{
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.radius = 10;
    rect_dsc.bg_opa = LV_OPA_COVER;
    rect_dsc.bg_grad.dir = LV_GRAD_DIR_HOR;
    rect_dsc.bg_grad.stops[0].color = lv_palette_main(LV_PALETTE_RED);
    rect_dsc.bg_grad.stops[1].color = lv_palette_main(LV_PALETTE_BLUE);
    rect_dsc.border_width = 2;
    rect_dsc.border_opa = LV_OPA_90;
    rect_dsc.border_color = lv_color_white();
    rect_dsc.shadow_width = 5;
    rect_dsc.shadow_ofs_x = 5;
    rect_dsc.shadow_ofs_y = 5;

    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_palette_main(LV_PALETTE_ORANGE);

    lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT)];

    lv_obj_t *canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_center(canvas);
    lv_canvas_fill_bg(canvas, lv_palette_lighten(LV_PALETTE_GREY, 3), LV_OPA_COVER);

    lv_canvas_draw_rect(canvas, 70, 60, 100, 70, &rect_dsc);

    lv_canvas_draw_text(canvas, 40, 20, 100, &label_dsc, "Some text on text canvas");

    /*Test the rotation. It requires another buffer where the original image is stored.
     *So copy the current image to buffer and rotate it to the canvas*/
    lv_color_t cbuf_tmp[CANVAS_WIDTH * CANVAS_HEIGHT];
    memcpy(cbuf_tmp, cbuf, sizeof(cbuf_tmp));
    lv_img_dsc_t img;
    img.data = (void *)cbuf_tmp;
    img.header.cf = LV_IMG_CF_TRUE_COLOR;
    img.header.w = CANVAS_WIDTH;
    img.header.h = CANVAS_HEIGHT;

    lv_canvas_fill_bg(canvas, lv_palette_lighten(LV_PALETTE_GREY, 3), LV_OPA_COVER);
    lv_canvas_transform(canvas, &img, 120, LV_IMG_ZOOM_NONE, 0, 0, CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, true);
}

static void anim_x_cb(void *var, int32_t v)
{
    lv_obj_set_x(var, v);
}
/*************************************************
 *  函数名称 :  anim_size_cb
 *  参    数 ： 无
 *  函数功能 ： 动画尺寸显示回调函数
 *************************************************/
static void anim_size_cb(void *var, int32_t v)
{
    lv_obj_set_size(var, v, v);
}
/*************************************************
 *  函数名称 :  anim_show_2
 *  参    数 ： 无
 *  函数功能 ： 按键联动，实现Label动画显示
 *************************************************/


lv_obj_t * anim_show_2()
{
    lv_obj_t *obj = lv_obj_create(lv_scr_act());                        // 创建一个对象
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), 0); // 设置背景颜色
    lv_obj_set_style_radius(obj, 0xFFF, 0);                             // 设置样式圆角
    lv_obj_align(obj, LV_ALIGN_BOTTOM_LEFT, 10, 0);                        // 居中样式

    lv_anim_t a;                                           // 创建动画样式
    lv_anim_init(&a);                                      // 初始化动画
    lv_anim_set_var(&a, obj);                              // 给动画设置一个变量
    lv_anim_set_values(&a, 5, 70);                         // 设置一个动画值
    lv_anim_set_time(&a, 5000);                            // 设置动画时间
    lv_anim_set_playback_delay(&a, 300);                   // 回放延时 使动画回放时，正向方向准备好了
    lv_anim_set_playback_time(&a, 3000);                   // 回放时间
    lv_anim_set_repeat_delay(&a, 5);                       // 重复延时
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); // 重复计数次数
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);     // 设置动画播放路径

    lv_anim_set_exec_cb(&a, anim_size_cb); // 给动画设置一个功能 回调函数为尺寸
    lv_anim_start(&a);                     // 开始动画
    lv_anim_set_exec_cb(&a, anim_x_cb);    // 给动画设置一个功能 回调函数为x轴值
    lv_anim_set_values(&a, 10, 240);       // 给动画设置一个值
    lv_anim_start(&a);                     // 开始动画
		return obj;
}

static void btn_event_cb_open(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
        printf("open\r\n");
        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "open: %d", cnt);
    }
}
static void btn_event_cb_close(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
        printf("close\r\n");
        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "close: %d", cnt);
    }
}
/**
 * Create a button with a label and react on click event.
 */
void lv_example_get_started_1(void)
{
    lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 120, 45);                            /*Set its position*/
    lv_obj_set_size(btn, 80, 50);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb_open, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "open");                     /*Set the labels text*/
    lv_obj_center(label);


    lv_obj_t * btn2 = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn2, 220, 45);                            /*Set its position*/
    lv_obj_set_size(btn2, 80, 50);                          /*Set its size*/
    lv_obj_add_event_cb(btn2, btn_event_cb_close, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label2 = lv_label_create(btn2);          /*Add a label to the button*/
    lv_label_set_text(label2, "close");                     /*Set the labels text*/
    lv_obj_center(label2);
}

lv_obj_t *objss = NULL;                        // 创建一个对象


static void btn_event_colorwheel(lv_event_t * e)
{
    // lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * wheel = lv_event_get_target(e);
    lv_color_t color = lv_colorwheel_get_rgb(wheel);
		lv_obj_set_style_bg_color(objss, color, 0); // 设置背景颜色
//    printf("color: %04X\r\n", color);

}

void lv_example_colorwheel_1(void)
{
    lv_obj_t * cw;

    cw = lv_colorwheel_create(lv_scr_act(), true);
    lv_obj_set_pos(cw, 200, 120); 
    lv_obj_set_size(cw, 100, 100);
    lv_obj_add_event_cb(cw, btn_event_colorwheel, LV_EVENT_ALL, NULL);
    // lv_obj_center(cw);
}

void test()
{
    LV_IMG_DECLARE(emoji_F617);
    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, &emoji_F617);
    lv_obj_set_pos(img, 35, 35); // 设置图片位置
    lv_img_set_angle(img, 30);   // 设置图片旋转角
    lv_img_set_zoom(img, 200);   // 设置图片缩放

    lv_example_slider_1();
    objss = anim_show_2();
    lv_example_get_started_1();
    lv_example_colorwheel_1();
    // lv_obj_t *parent = lv_obj_create(lv_scr_act());
    // lv_obj_set_size(parent, 50, 50);
    // lv_obj_set_style_bg_color(parent, lv_palette_main(LV_PALETTE_BLUE), 0);
    // lv_obj_align(parent, LV_ALIGN_BOTTOM_MID, 0, -50);

    // lv_obj_t *child = lv_obj_create(parent);
    // lv_obj_set_size(child, 20, 20);
    // lv_obj_set_style_bg_color(child, lv_palette_main(LV_PALETTE_GREEN), 0);
    // lv_obj_align(child, LV_ALIGN_CENTER, 0, 0);
}
