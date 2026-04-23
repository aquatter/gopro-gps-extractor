#pragma once
#include <cstdint>
#include <memory>
#include <utility>
#include <windows.h>

class D3DWindow {
public:
  D3DWindow();
  ~D3DWindow();

  void show() const;
  void update() const;
  float scale() const noexcept;
  void init_imgui();

  void reset_resize() const noexcept;

  std::pair<uint32_t, uint32_t> get_resized() const noexcept;

  HWND hwnd() const noexcept;

private:
  struct impl;
  std::unique_ptr<impl> pimpl_;
};