#pragma once
#include <memory>
#include <windows.h>

class D3DDevice {
public:
  D3DDevice(HWND hwnd);
  ~D3DDevice();

  void CreateRenderTarget();
  void CleanupRenderTarget();
  bool occluded();
  void resize(uint32_t width, uint32_t height);
  void init_imgui();
  void render();

private:
  struct impl;
  std::unique_ptr<impl> pimpl_;
};