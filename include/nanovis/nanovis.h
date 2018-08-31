#pragma once

#include <string>
#include <memory>
#include <functional>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace nanovis {

class NanoVis {
    struct NanoVisImpl; // pimpl
public:
    NanoVis(const std::string &title = "NanoVis", int width = 1280, int height = 720);
    virtual ~NanoVis();

    void show();
    void refresh();

    int width() const;
    int height() const;

    void add_button(const std::string &title, const std::string &name, const std::function<void()> &callback = {});

    void add_toggle(const std::string &title, const std::string &name, const std::function<void(bool)> &callback = {});
    void add_toggle(const std::string &title, const std::string &name, bool &value, const std::function<void(bool)> &callback = {});

    void add_repeat(const std::string &title, const std::string &name, const std::function<bool()> &callback = {});

    void add_graph(const std::string &title, const std::string &name, double &value, const double &max_value, const double &min_value);
    void add_graph(const std::string &title, const std::string &name, std::vector<double> &values, const double &max_value, const double &min_value);

    void add_image(const std::string &title, const std::string &name, cv::Mat &image);

    template <typename T>
    void notify(const T &value) {
        broadcast((const void *)(&value));
    }

    Eigen::Matrix4f proj_matrix(float near = 1.0e-1f, float far = 1.0e3f) const;
    Eigen::Matrix4f view_matrix() const;
    Eigen::Matrix4f world_matrix() const;
    Eigen::Matrix4f model_view_proj() const;
    float world_scale() const;

protected:
    virtual void draw();

private:
    void broadcast(const void *value, bool global = true);
    std::unique_ptr<NanoVisImpl> impl;
    friend class NanoVisSystem;
};

void main(int interval = 50);

}
