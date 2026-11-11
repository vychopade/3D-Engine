#pragma once

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ui {

struct State {
    bool hovered = false;
    bool used = false;
    bool absorb = false;
    int active = 0;
    int hot = 0;
    int id = 1;

    void begin() {
        hovered = false;
        used = false;
        absorb = false;
        hot = 0;
        id = 1;
    }

    void end() {
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            active = 0;
        }
    }

    int next() { return id++; }

    bool over(Rectangle r) const { return CheckCollisionPointRec(GetMousePosition(), r); }

    void mark(Rectangle r) {
        if (over(r)) {
            hovered = true;
        }
    }
};

inline void panel(Rectangle r, Color color) {
    DrawRectangleRec(r, color);
    DrawRectangleLinesEx(r, 1, Color{255, 255, 255, 18});
}

inline void label(int x, int y, const char* text, Color color = LIGHTGRAY, int size = 16) {
    DrawText(text, x, y, size, color);
}

inline bool button(State& ui, Rectangle r, const char* text, bool selected = false) {
    const int id = ui.next();
    ui.mark(r);
    const bool hover = ui.over(r);
    if (hover) {
        ui.hot = id;
    }

    Color bg = selected ? Color{52, 120, 196, 255} : Color{42, 46, 60, 255};
    if (hover) {
        bg = selected ? Color{66, 140, 220, 255} : Color{58, 64, 82, 255};
    }
    DrawRectangleRounded(r, 0.12f, 4, bg);
    if (selected) {
        DrawRectangleLinesEx(r, 2, Color{140, 200, 255, 200});
    }

    const int tw = MeasureText(text, 16);
    DrawText(text, static_cast<int>(r.x + (r.width - static_cast<float>(tw)) * 0.5f),
             static_cast<int>(r.y + r.height * 0.5f - 8.0f), 16, RAYWHITE);

    if (!ui.absorb && hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        ui.active = id;
        ui.used = true;
        return true;
    }
    return false;
}

inline bool toggle(State& ui, Rectangle r, const char* text, bool value) {
    if (button(ui, r, text, value)) {
        return !value;
    }
    return value;
}

inline float slider(State& ui, Rectangle r, float value, float min_v, float max_v, const char* caption) {
    const int id = ui.next();
    ui.mark(r);
    const bool hover = ui.over(r);
    if (hover) {
        ui.hot = id;
    }
    if (!ui.absorb && hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        ui.active = id;
        ui.used = true;
    }

    DrawText(caption, static_cast<int>(r.x), static_cast<int>(r.y - 16), 14, GRAY);
    DrawRectangleRec(r, Color{28, 30, 40, 255});
    const float t = std::clamp((value - min_v) / (max_v - min_v), 0.0f, 1.0f);
    DrawRectangleRec({r.x, r.y, r.width * t, r.height}, Color{66, 165, 245, 220});
    DrawRectangleLinesEx(r, 1, Color{255, 255, 255, 28});

    if (ui.active == id && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        ui.used = true;
        const float x = std::clamp((GetMousePosition().x - r.x) / r.width, 0.0f, 1.0f);
        value = min_v + x * (max_v - min_v);
    }

    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(value));
    const int tw = MeasureText(buf, 14);
    DrawText(buf, static_cast<int>(r.x + r.width - static_cast<float>(tw)), static_cast<int>(r.y - 16), 14,
             LIGHTGRAY);
    return value;
}

inline void text_field(State& ui, Rectangle r, char* buf, int max_len, bool& focused) {
    const int id = ui.next();
    ui.mark(r);
    const bool hover = ui.over(r);
    if (!ui.absorb && hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        focused = true;
        ui.active = id;
        ui.used = true;
    }

    DrawRectangleRec(r, Color{28, 30, 40, 255});
    DrawRectangleLinesEx(r, focused ? 2 : 1, focused ? Color{66, 165, 245, 255} : Color{255, 255, 255, 28});

    if (focused) {
        int c = GetCharPressed();
        while (c > 0) {
            const int len = static_cast<int>(std::strlen(buf));
            const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' ||
                            c == '-' || c == ' ';
            if (ok && len < max_len - 1) {
                buf[len] = static_cast<char>(c);
                buf[len + 1] = '\0';
            }
            c = GetCharPressed();
        }
        if (IsKeyPressed(KEY_BACKSPACE)) {
            const int len = static_cast<int>(std::strlen(buf));
            if (len > 0) {
                buf[len - 1] = '\0';
            }
        }
    }

    DrawText(buf, static_cast<int>(r.x + 8), static_cast<int>(r.y + r.height * 0.5f - 8.0f), 16, RAYWHITE);
    if (focused && static_cast<int>(GetTime() * 2.0) % 2 == 0) {
        const int tw = MeasureText(buf, 16);
        DrawRectangle(static_cast<int>(r.x) + 10 + tw, static_cast<int>(r.y) + 6, 2, static_cast<int>(r.height) - 12,
                      RAYWHITE);
    }
}

}  // namespace ui
