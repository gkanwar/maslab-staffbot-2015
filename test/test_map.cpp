#include "map.h"

#include "gtest/gtest.h"

TEST(MapTest, TestWallRaster) {
  Map m({Wall(5.0, 5.0, 10.0, 10.0)}, {});

  EXPECT_EQ(Map::NONE, m.getMapElement(0.0, 0.0));
  EXPECT_EQ(Map::NONE, m.getMapElement(5.5, 5.0));

  EXPECT_EQ(Map::WALL, m.getMapElement(5.0, 5.0));
  EXPECT_EQ(Map::WALL, m.getMapElement(10.0, 10.0));
  EXPECT_EQ(Map::WALL, m.getMapElement(7.0, 7.0));
}
