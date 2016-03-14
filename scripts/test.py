#!/usr/bin/env python


from xdo import xdo
xdo = xdo()
win_id = xdo.select_window_with_click()
xdo.enter_text_window(win_id, 'Python rocks!')
