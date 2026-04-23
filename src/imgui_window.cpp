#include <atomic>
#include <chrono>
#include <d3ddevice.hpp>
#include <d3dwindow.hpp>
#include <fmt/chrono.h>
#include <fmt/format.h>
#include <gpmf_parser.hpp>
#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>
#include <imgui_window.hpp>
#include <memory>
#include <mutex>
#include <portable-file-dialogs.h>
#include <string_view>
#include <thread>
#include <vector>

struct ImGUIWindow::impl {
  impl() : done_{false}, button_disabled_{false} {
    d3d_window_ = std::make_unique<D3DWindow>();
    d3d_device_ = std::make_unique<D3DDevice>(d3d_window_->hwnd());
    d3d_window_->show();
    d3d_window_->update();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io{ImGui::GetIO()};
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGuiStyle &style{ImGui::GetStyle()};
    style.ScaleAllSizes(d3d_window_->scale());
    style.FontScaleDpi = d3d_window_->scale();

    d3d_window_->init_imgui();
    d3d_device_->init_imgui();
  }

  void run() {
    while (!done_) {
      process_messages();

      if (done_) {
        break;
      }

      if (d3d_device_->occluded()) {
        continue;
      }

      check_resize();
      new_frame();
      draw_controls();
      draw_log();
      d3d_device_->render();
    }
  }

  void draw_controls() {
    ImGui::Begin("GoPro GPS Extractor");

    ImGui::Text("Choose folder with *.MP4 records");

    ImGui::BeginDisabled(button_disabled_.load());
    if (ImGui::Button("Select Folder")) {
      const auto selection{pfd::select_folder("Select a folder").result()};
      if (!selection.empty()) {
        parse(selection);
      }
    }
    ImGui::EndDisabled();

    ImGui::End();
  }

  void draw_log() {

    ImGui::SetNextWindowSize({520, 600}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({400, 60}, ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Log")) {
      ImGui::End();
      return;
    }

    if (ImGui::BeginChild("ScrollingRegion", {0, 0},
                          ImGuiChildFlags_NavFlattened,
                          ImGuiWindowFlags_HorizontalScrollbar)) {

      const auto log_entries{get_log()};

      for (auto &&entry : log_entries) {
        ImVec4 clr{};
        std::string prefix{};

        switch (entry.severity_) {
        case LogEntry::Severity::Info:
          clr = {0.412f, 0.788f, 0.525f, 1.0f};
          prefix = "[INFO]";
          break;
        case LogEntry::Severity::Warning:
          clr = {0.788f, 0.745f, 0.411f, 1.0f};
          prefix = "[WARNING]";
          break;
        //   201, 105, 108
        case LogEntry::Severity::Error:
          clr = {0.788f, 0.411f, 0.423f, 1.0f};
          prefix = "[ERROR]";
          break;
        }

        ImGui::PushStyleColor(ImGuiCol_Text, clr);
        ImGui::TextUnformatted(fmt::format("{} {:%Y-%m-%d %H:%M:%S} {}", prefix,
                                           entry.time_, entry.message_)
                                   .c_str());
        ImGui::PopStyleColor();
      }

      ImGui::SetScrollHereY(1.0f);
      ImGui::EndChild();
    }

    ImGui::End();
  }

  void new_frame() {
    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
  }

  void process_messages() {
    MSG msg;
    while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
      ::TranslateMessage(&msg);
      ::DispatchMessage(&msg);

      if (msg.message == WM_QUIT) {
        done_ = true;
      }
    }
  }

  void check_resize() {
    const auto [w, h] = d3d_window_->get_resized();
    if (w != 0 and h != 0) {
      d3d_device_->resize(w, h);
      d3d_window_->reset_resize();
    }
  }

  void parse(const std::string path) {
    button_disabled_.store(true);

    wt_ = std::thread([this, path]() {
      GPMFParser parser{GPMFParserSettings{
          .paths_to_mp4_ = {path},
          .output_path_ = path,
          .log_callback_ =
              [this](LogEntry::Severity severity, const std::string_view msg) {
                push_to_log(severity, msg);
              },
          .end_callback_ = [this]() { button_disabled_.store(false); }}};

      parser.parse();
    });
  }

  void push_to_log(LogEntry::Severity severity, const std::string_view msg) {
    std::lock_guard<std::mutex> lock{log_protector_};
    log_entries_.emplace_back(std::chrono::system_clock::now(), severity,
                              msg.data());
  }

  std::vector<LogEntry> get_log() {
    std::lock_guard<std::mutex> lock{log_protector_};
    return log_entries_;
  }

  ~impl() {
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    d3d_device_.reset();
    d3d_window_.reset();

    if (wt_.joinable()) {
      wt_.join();
    }
  }

  std::unique_ptr<D3DWindow> d3d_window_;
  std::unique_ptr<D3DDevice> d3d_device_;
  bool done_;
  std::thread wt_;
  std::vector<LogEntry> log_entries_;
  std::mutex log_protector_;
  std::atomic_bool button_disabled_;
};

ImGUIWindow::ImGUIWindow() : pimpl_{std::make_unique<impl>()} {}
void ImGUIWindow::run() { pimpl_->run(); }
ImGUIWindow::~ImGUIWindow() = default;