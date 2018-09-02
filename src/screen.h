#pragma once

#include <list>
#include <nanogui/nanogui.h>

class NanoVisWindow;

class NanoVisScreen : public nanogui::Screen {
    struct IntervalJob {
        int refresh;
        int remaining;
        std::function<bool()> callback;
    };

  public:
    NanoVisScreen(NanoVisWindow *window, const nanogui::Vector2i &size, const std::string &caption);

    void drawAll() override;
    void drawContents() override;

    void setInterval(const std::function<bool()> callback, int refresh = 50);

    bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) override;
    bool mouseMotionEvent(const nanogui::Vector2i &p, const nanogui::Vector2i &rel, int button, int modifiers) override;
    bool mouseDragEvent(const nanogui::Vector2i &p, const nanogui::Vector2i &rel, int button, int modifiers) override;
    bool mouseEnterEvent(const nanogui::Vector2i &p, bool enter) override;
    bool scrollEvent(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) override;

    void set_position(int x, int y);

  private:
    NanoVisWindow *window;
    double last_refresh_time = 0;
    std::list<IntervalJob> interval_jobs;
    void notifyAllJobs();
};
