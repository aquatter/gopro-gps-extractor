#pragma once
#include <d3d11.h>

class D3DWindow {
public:
  D3DWindow();

private:
  void CreateRenderTarget();
  bool CreateDeviceD3D(HWND hWnd);
  void CleanupRenderTarget();

  ID3D11Device *g_pd3dDevice = nullptr;
  ID3D11DeviceContext *g_pd3dDeviceContext = nullptr;
  IDXGISwapChain *g_pSwapChain = nullptr;
  bool g_SwapChainOccluded = false;
  UINT g_ResizeWidth = 0, g_ResizeHeight = 0;
  ID3D11RenderTargetView *g_mainRenderTargetView = nullptr;
};
