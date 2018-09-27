#pragma once

#include <unordered_map>
#include <functional>
#include <nanogui/nanogui.h>
#include <opencv2/opencv.hpp>

class NanoVisScreen;

class NanoVisWindow {
    friend class NanoVisScreen;

  public:
    NanoVisWindow(const std::string &title, int width, int height);
    virtual ~NanoVisWindow();

    void show();
    void refresh();

    int width() const;
    int height() const;

    void add_button(const std::string &title, const std::string &name, const std::function<void()> &callback);

    void add_toggle(const std::string &title, const std::string &name, const std::function<void(bool)> &callback);
    void add_toggle(const std::string &title, const std::string &name, bool &value, const std::function<void(bool)> &callback);

    void add_repeat(const std::string &title, const std::string &name, const std::function<bool()> &callback);

    void add_graph(const std::string &title, const std::string &name, const double &value, const double &max_value, const double &min_value, const Eigen::Vector3d &color);
    void add_graph(const std::string &title, const std::string &name, const std::vector<double> &values, const double &max_value, const double &min_value, const Eigen::Vector3d &color);

    void add_image(const std::string &title, const std::string &name, const cv::Mat &image);

    void add_widget(const std::string &title, nanogui::Widget *widget);
    void add_widget(const std::string &title, nanogui::Widget *widget_left, nanogui::Widget *widget_right);

    void set_timeout(int refresh, const std::function<bool()> &callback);
    void set_camera(const Eigen::Vector3f &position, float roll, float yaw, float pitch) {
        viewport_xyz = position;
        viewport_ryp = {roll, yaw, pitch};
    }

    nanogui::Window *panel(const std::string &title);

    template <typename T>
    void add_variable(T &value, const std::function<void()> &handler) {
        subscribe((void *)(&value), handler);
    }

    template <typename T>
    void notify(const T &value) {
        broadcast((const void *)(&value));
    }

    void subscribe(void *value, const std::function<void()> &subscriber);
    void broadcast(const void *value);

    Eigen::Matrix4f proj_matrix(float near, float far) const;
    Eigen::Matrix4f view_matrix() const;
    Eigen::Matrix4f world_matrix() const;
    float world_scale() const;

    const Eigen::Vector3f &pickup_point() const;

    void set_position(int x, int y);

  protected:
    nanogui::AdvancedGridLayout *layout(const std::string &title);
    virtual void draw();

  protected:
    virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers);
    virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers);
    virtual bool mouseDragEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers);
    virtual bool mouseEnterEvent(const Eigen::Vector2i &p, bool enter);
    virtual bool mouseScrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel);

  private:
    Eigen::Vector2i viewport_cursor_old;
    Eigen::Vector3f viewport_pickup_point;
    Eigen::Vector3f viewport_xyz, viewport_xyz_old;
    Eigen::Vector3f viewport_ryp, viewport_ryp_old;
    float viewport_scale;
    bool viewport_rotation_mode = false;
    bool viewport_translation_mode = false;

  private:
    nanogui::ref<NanoVisScreen> m_screen;
    std::unordered_map<std::string, nanogui::Window *> m_panels;

  private:
    std::unordered_map<const void *, std::unordered_map<nanogui::Widget *, std::function<void()>>> m_subscribers;
};
