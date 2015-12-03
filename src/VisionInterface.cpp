#include <iostream>

#include "VisionInterface.h"

VisionInterface* VisionInterface::m_instance_ = NULL;

VisionInterface::VisionInterface(QObject *parent)
  : QObject(parent),
    ros_interface_(ROSinterface::getInstance()) {
  connect(
          ros_interface_, SIGNAL(setHueMin(int)),
          this, SLOT());
}

VisionInterface* VisionInterface::getInstance() {
  if (!m_instance_) {
    m_instance_ = new VisionInterface;
  }
  return m_instance_;
}

/*
 * SLOTS
 */
void VisionInterface::onHueMinChanged(int value) {
  std::cout << "Hue Min: " << value << std::endl;
}
