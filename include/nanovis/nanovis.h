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

    void add_graph(const std::string &title, const std::string &name, const double &value, const double &max_value, const double &min_value, const Eigen::Vector3d &color = {0.6, 0.5, 0.0});
    void add_graph(const std::string &title, const std::string &name, const std::vector<double> &values, const double &max_value, const double &min_value, const Eigen::Vector3d &color = {0.6, 0.5, 0.0});

    void add_image(const std::string &title, const std::string &name, const cv::Mat &image);

    void add_points(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &color = {1.0, 1.0, 0.0}, const double &point_size = 3.0);
    void add_points(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, const double &point_size = 3.0);

    void add_path(const std::vector<Eigen::Vector3d> &vertices, const Eigen::Vector3d &color = {1.0, 1.0, 0.0});
    void add_path(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3d> &colors);

    void set_timeout(int refresh, const std::function<bool()> &callback = {});

    void set_grid_visible(bool visible);
    void set_camera(const Eigen::Vector3d &position, double roll, double yaw, double pitch);

    template <typename T>
    void notify(const T &value) {
        broadcast((const void *)(&value));
    }

    Eigen::Matrix4f proj_matrix(float near = 1.0e-1f, float far = 1.0e3f) const;
    Eigen::Matrix4f view_matrix() const;
    Eigen::Matrix4f world_matrix() const;
    Eigen::Matrix4f model_view_proj() const;
    float world_scale() const;

    void set_position(int x, int y);

  protected:
    virtual void draw();

  private:
    void broadcast(const void *value, bool global = true);
    std::unique_ptr<NanoVisImpl> impl;
    friend class NanoVisSystem;
};

void main(int interval = 1);

} // namespace nanovis
