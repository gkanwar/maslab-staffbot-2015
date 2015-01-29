#include "map.h"

#include "gtest/gtest.h"

TEST(MapTest, TestWallRaster) {
  Map m({Wall(5.0, 5.0, 10.0, 10.0)}, {}, RobotPose(1, 2, PI));

  EXPECT_EQ(Map::NONE, m.getMapElement(0.0, 0.0));
  EXPECT_EQ(Map::NONE, m.getMapElement(5.5, 5.0));

  EXPECT_EQ(Map::WALL, m.getMapElement(5.0, 5.0));
  EXPECT_EQ(Map::WALL, m.getMapElement(10.0, 10.0));
  EXPECT_EQ(Map::WALL, m.getMapElement(7.0, 7.0));

  EXPECT_DOUBLE_EQ(1, m.getInitPose().x);
  EXPECT_DOUBLE_EQ(2, m.getInitPose().y);
  EXPECT_DOUBLE_EQ(PI, m.getInitPose().theta);
}
