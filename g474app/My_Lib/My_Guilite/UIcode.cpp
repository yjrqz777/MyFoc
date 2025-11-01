#define GUILITE_ON  //Do not define this macro once more!!!
#include "GuiLite.h"
#include <stdlib.h>
#include <stdio.h>
#include "adc.h"
#define UI_WIDTH 240
#define UI_HEIGHT 135

enum WND_ID
{
	ID_ROOT = 1,
	ID_LABEL1,
	ID_LABEL2,
//	ID_LABEL3,
	ID_BUTTON1,
	ID_BUTTON2,
//	ID_BUTTON3
};




class c_myUI : public c_wnd
{
	virtual void on_init_children()
	{
		((c_button*)get_wnd_ptr(ID_BUTTON1))->set_on_click((WND_CALLBACK)&c_myUI::on_clicked);
		((c_button*)get_wnd_ptr(ID_BUTTON2))->set_on_click((WND_CALLBACK)&c_myUI::on_clicked);
		
	}
	virtual void on_paint(void)
	{
		c_rect rect;
		get_screen_rect(rect);
		m_surface->fill_rect(rect, GL_RGB(0, 0, 0), m_z_order);
	}
	void on_clicked(int ctrl_id, int param) {
		static int sum1, sum2, sum3;
		static char str1[8], str2[8], str3[8];
		c_button* button = (c_button*)get_wnd_ptr(ctrl_id);
		c_label* lable = (c_label*)get_wnd_ptr(ID_LABEL1);
		c_label* lable2 = (c_label*)get_wnd_ptr(ID_LABEL2);
		switch (ctrl_id)
		{
		case ID_BUTTON1:
			sprintf(str1, "%d", ++sum1);
			printf(str1);
			button->set_str(str1);
			lable->set_str(str1);
		break;
		case ID_BUTTON2:
			sprintf(str2, "%d", ++sum2);
			button->set_str(str2);
		lable2->show_window();
			break;
//		case ID_BUTTON3:
//			sprintf(str3, "%d", ++sum3);
//			button->set_str(str3);
//			break;
		}
		lable->show_window();
		button->show_window();
	}
};


//class adc_test : public c_myUI
//{
//public:
//	virtual void updata()
//	{
//		c_label* lable2 = (c_label*)get_wnd_ptr(ID_LABEL2);
//		lable2->set_str("000");
//		lable2->show_window();
//	}
//};
//static adc_test adc2;

void update_label2_with_adc_value(c_label* label)
{
    static char adc_str[8];
		uint16_t adc = 0;
		HAL_ADC_Start(&hadc2);
		adc = HAL_ADC_GetValue(&hadc2);
//    uint16_t adc_value = ADC_Read();
    sprintf(adc_str, "%d", adc);
		printf(adc_str);
//    label->set_str(adc_str);
//    label->show_window();  // Ë¢ÐÂÏÔÊ¾
}


//////////////////////// layout UI ////////////////////////
static c_myUI s_myUI;
static c_label s_label1, s_label2, s_label3;
static c_button s_button1, s_button2, s_button3;

extern "C" void adc_test2()
{
	
//	adc2.updata();
	
	update_label2_with_adc_value(&s_label2);
//	adc2.show_window();
	
	
}


static WND_TREE s_myUI_children[] =
{
	{&s_label1, ID_LABEL1, "00", 20, 20, 80, 20, NULL},
	{&s_label2, ID_LABEL2, "asdasd", 20, 60, 80, 20, NULL},
//	{&s_label3, ID_LABEL3, "s: click", 20, 260, 80, 40, NULL},

	{&s_button1, ID_BUTTON1, "0", 100, 20, 40, 30, NULL},
	{&s_button2, ID_BUTTON2, "0", 100, 60, 40, 30, NULL},
//	{&s_button3, ID_BUTTON3, "0", 140, 260, 80, 40, NULL},
	{ NULL,0,0,0,0,0,0 }
};
//////////////////////// start UI ////////////////////////
extern const LATTICE_FONT_INFO Consolas_28;
void load_resource()
{
	c_theme::add_font(FONT_DEFAULT, &Consolas_28);
	//for button
	c_theme::add_color(COLOR_WND_FONT, GL_RGB(255, 255, 243));
	c_theme::add_color(COLOR_WND_NORMAL, GL_RGB(59, 75, 94));
	c_theme::add_color(COLOR_WND_PUSHED, GL_RGB(33, 42, 53));
	c_theme::add_color(COLOR_WND_FOCUS, GL_RGB(78, 198, 76));
	c_theme::add_color(COLOR_WND_BORDER, GL_RGB(46, 59, 73));
}

static c_display* s_display;
static c_surface* s_surface;
void create_ui(void* phy_fb, int screen_width, int screen_height, int color_bytes, struct DISPLAY_DRIVER* driver)
{
	load_resource();
	static c_surface surface(UI_WIDTH, UI_HEIGHT, color_bytes, Z_ORDER_LEVEL_0);
	static c_display display(phy_fb, screen_width, screen_height, &surface, driver);
	s_surface = &surface;
	s_display = &display;
	s_myUI.set_surface(s_surface);
	s_myUI.connect(NULL, ID_ROOT, 0, 0, 0, UI_WIDTH, UI_HEIGHT, s_myUI_children);
	s_myUI.show_window();
}

//////////////////////// interface for all platform ////////////////////////
extern "C" void startHelloKeypad(void* phy_fb, int width, int height, int color_bytes, struct DISPLAY_DRIVER* driver)
{
	create_ui(phy_fb, width, height, color_bytes, driver);
}

extern "C" void sendTouch2HelloKeypad(int x, int y, bool is_down)
{
	is_down ? s_myUI.on_touch(x, y, TOUCH_DOWN) : s_myUI.on_touch(x, y, TOUCH_UP);
}

extern "C" void sendKey2HelloKeypad(unsigned int key)
{
	s_myUI.on_navigate(NAVIGATION_KEY(key));
	
}

extern void* getUiOfHelloKeypad(int* width, int* height, bool force_update = false)
{
	if (s_display)
	{
		return s_display->get_updated_fb(width, height, force_update);
	}
	return NULL;
}

int captureUiOfHelloKeypad()
{
	return s_display->snap_shot("snap_short.bmp");
}
