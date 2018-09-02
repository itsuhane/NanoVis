#include "screen.h"
#include "window.h"

using namespace nanogui;

NanoVisScreen::NanoVisScreen(NanoVisWindow *window, const Vector2i &size, const std::string &caption) :
    window(window), Screen(size, caption) {
}

void NanoVisScreen::drawAll() {
    notifyAllJobs();
    Screen::drawAll();
}

void NanoVisScreen::drawContents() {
    glfwMakeContextCurrent(mGLFWWindow);
    window->draw();
}

void NanoVisScreen::setInterval(const std::function<bool()> callback, int refresh) {
    interval_jobs.emplace_back();
    IntervalJob &job = interval_jobs.back();
    job.refresh = job.remaining = refresh;
    job.callback = callback;
}

bool NanoVisScreen::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {
    if (Screen::mouseButtonEvent(p, button, down, modifiers)) {
        return true;
    }
    return window->mouseButtonEvent(p, button, down, modifiers);
}

bool NanoVisScreen::mouseMotionEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers) {
    if (Screen::mouseMotionEvent(p, rel, button, modifiers)) {
        return true;
    }
    return window->mouseMotionEvent(p, rel, button, modifiers);
}

bool NanoVisScreen::mouseDragEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers) {
    if (Screen::mouseDragEvent(p, rel, button, modifiers)) {
        return true;
    }
    return window->mouseDragEvent(p, rel, button, modifiers);
}

bool NanoVisScreen::mouseEnterEvent(const Vector2i &p, bool enter) {
    if (Screen::mouseEnterEvent(p, enter)) {
        return true;
    }
    return window->mouseEnterEvent(p, enter);
}

bool NanoVisScreen::scrollEvent(const Vector2i &p, const Vector2f &rel) {
    if (Screen::scrollEvent(p, rel)) {
        return true;
    }
    return window->mouseScrollEvent(p, rel);
}

void NanoVisScreen::set_position(int x, int y) {
    glfwSetWindowPos(mGLFWWindow, x, y);
}

void NanoVisScreen::notifyAllJobs() {
    double current_refresh_time = glfwGetTime();
    int time_elapsed_ms = (int)(1000 * (current_refresh_time - last_refresh_time));
    last_refresh_time = current_refresh_time;
    for (auto &job : interval_jobs) {
        job.remaining -= time_elapsed_ms;
        if (job.remaining <= 0) {
            if (job.callback) {
                if (!job.callback()) {
                    job.callback = {};
                }
            }
            job.remaining += (1 + (-job.remaining) / job.refresh) * job.refresh;
        }
    }
    interval_jobs.remove_if([](const IntervalJob &job) {
        return !job.callback;
    });
}
