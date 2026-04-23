#pragma once
#include <memory>

class ImGUIWindow {
public:
  ImGUIWindow();
  void run();
  ~ImGUIWindow();

private:
  struct impl;
  std::unique_ptr<impl> pimpl_;
};