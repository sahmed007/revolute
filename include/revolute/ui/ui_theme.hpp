#pragma once

namespace revolute::ui {
// Rasterize at display density while preserving an 18 px logical font size.
void load_ui_font(float display_scale);

void apply_ui_theme();
} // namespace revolute::ui
