#include <d3d11.h>
#include <d3dwindow.hpp>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>
#include <memory>
#include <stdexcept>

static UINT g_ResizeWidth{0};
static UINT g_ResizeHeight{0};

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd,
                                                             UINT msg,
                                                             WPARAM wParam,
                                                             LPARAM lParam);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
  if (auto res{ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)};
      res != 0) {
    return res;
  }

  switch (msg) {
  case WM_SIZE:
    if (wParam == SIZE_MINIMIZED) {
      return 0;
    }
    g_ResizeWidth = (UINT)LOWORD(lParam); // Queue resize
    g_ResizeHeight = (UINT)HIWORD(lParam);
    return 0;
  case WM_SYSCOMMAND:
    if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
    {
      return 0;
    }
    break;
  case WM_DESTROY:
    ::PostQuitMessage(0);
    return 0;
  }

  return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}

struct D3DWindow::impl {
  impl() {
    ImGui_ImplWin32_EnableDpiAwareness();
    main_scale_ = ImGui_ImplWin32_GetDpiScaleForMonitor(
        ::MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));

    wc_ = {sizeof(wc_), CS_CLASSDC,   WndProc,
           0L,          0L,           GetModuleHandle(nullptr),
           nullptr,     nullptr,      nullptr,
           nullptr,     L"GoProExtr", nullptr};

    ::RegisterClassExW(&wc_);

    hwnd_ = ::CreateWindowW(wc_.lpszClassName, L"GoPro GPS Extractor",
                            WS_OVERLAPPEDWINDOW, 100, 100,
                            static_cast<int>(1000 * main_scale_),
                            static_cast<int>(800 * main_scale_), nullptr,
                            nullptr, wc_.hInstance, nullptr);

    if (hwnd_ == nullptr) {
      throw std::runtime_error{"unable to create directx window"};
    }
  }

  ~impl() {
    ::DestroyWindow(hwnd_);
    ::UnregisterClassW(wc_.lpszClassName, wc_.hInstance);
  }

  float main_scale_;
  WNDCLASSEXW wc_;
  HWND hwnd_;
};

D3DWindow::D3DWindow() : pimpl_{std::make_unique<impl>()} {}
HWND D3DWindow::hwnd() const noexcept { return pimpl_->hwnd_; }
void D3DWindow::show() const { ::ShowWindow(pimpl_->hwnd_, SW_SHOWDEFAULT); }
void D3DWindow::update() const { ::UpdateWindow(pimpl_->hwnd_); }

float D3DWindow::scale() const noexcept { return pimpl_->main_scale_; }

void D3DWindow::init_imgui() { ImGui_ImplWin32_Init(pimpl_->hwnd_); }

void D3DWindow::reset_resize() const noexcept {
  g_ResizeWidth = 0;
  g_ResizeHeight = 0;
}

std::pair<uint32_t, uint32_t> D3DWindow::get_resized() const noexcept {
  return {g_ResizeWidth, g_ResizeHeight};
}

D3DWindow::~D3DWindow() = default;
