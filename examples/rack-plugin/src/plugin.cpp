#include "plugin.hpp"

Plugin* pluginInstance;

void init(Plugin* p) {
  pluginInstance = p;
  p->addModel(modelStrike);
  p->addModel(modelHaze);
  p->addModel(modelKit);
  p->addModel(modelDrift);
  p->addModel(modelPalette);
  p->addModel(modelBridge);
}
