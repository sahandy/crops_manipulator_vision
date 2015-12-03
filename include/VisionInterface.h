#ifndef VISION_INTERFACE_H
#define VISION_INTERFACE_H

#include <QObject>

#include "ROSinterface.hpp"

class VisionInterface : public QObject {
Q_OBJECT

public:
  static VisionInterface* getInstance();

private:
  explicit VisionInterface(QObject *parent=0);
  ~VisionInterface() {}

  static VisionInterface* m_instance_;
  ROSinterface *ros_interface_;

private slots:
  void onHueMinChanged(int);
};

#endif // VISION_INTERFACE_H
