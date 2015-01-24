#include "map.h"

#include "gtest/gtest.h"

TEST(MapTest, TestEmptyMap) {
  Map m({}, {});
  EXPECT_EQ(Map::NONE, m.getMapElement(0.0, 0.0));
  EXPECT_EQ(Map::NONE, m.getMapElement(50.0, 50.0));
  EXPECT_EQ(Map::NONE, m.getMapElement(0.0, 50.0));
  EXPECT_EQ(Map::NONE, m.getMapElement(50.0, 0.0));
}

TEST(MapTest, TestWallRaster) {
  Map m({Wall(5.0, 5.0, 10.0, 10.0)}, {});

  EXPECT_EQ(Map::NONE, m.getMapElement(0.0, 0.0));
  EXPECT_EQ(Map::NONE, m.getMapElement(5.5, 5.0));

  EXPECT_EQ(Map::WALL, m.getMapElement(5.0, 5.0));
  EXPECT_EQ(Map::WALL, m.getMapElement(10.0, 10.0));
  EXPECT_EQ(Map::WALL, m.getMapElement(7.0, 7.0));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
