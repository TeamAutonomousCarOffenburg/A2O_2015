#include "gtest/gtest.h"

#include "worldmodel/impl/WorldState.h"

using namespace A2O;
using namespace std;

namespace {

	class WorldModelTest: public ::testing::Test {
		protected:

			WorldModelTest() {
			}

			virtual ~WorldModelTest() {
			}


			virtual void SetUp() {
			}

			virtual void TearDown() {
			}

	};

	TEST_F(WorldModelTest, testisDirectAncestor) {
		WorldState::ConstPtr state1(new WorldState(NULL));

		WorldState::ConstPtr state2 = state1->getChild();
		WorldState::ConstPtr state2b = state1->getChild();
		WorldState::ConstPtr state3 = state2->getChild();
		WorldState::ConstPtr state3b = state2b->getChild();

		ASSERT_FALSE(state2->isDirectAncestor(state1));
		ASSERT_TRUE(state1->isDirectAncestor(state2));
		ASSERT_TRUE(state1->isDirectAncestor(state2b));
		ASSERT_TRUE(state1->isDirectAncestor(state3));
		ASSERT_TRUE(state1->isDirectAncestor(state3b));
	}
	
	TEST_F(WorldModelTest, testIntersection) {
	
		std::vector<int> v1;
		v1.push_back(16);
		v1.push_back(18);

		
		std::vector<int> v2;
		v2.push_back(20);
		v2.push_back(16);

		std::sort(v1.begin(), v1.end());
		std::sort(v2.begin(), v2.end());

		std::vector<int> intersection;
		std::set_intersection (v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(intersection));	

		
		ASSERT_TRUE(intersection.size() == 1);
		ASSERT_TRUE(intersection[0] == 16);
	}

	TEST_F(WorldModelTest, testfindCommonAncestor) {

		WorldState::ConstPtr state1(new WorldState(NULL));
		WorldState::ConstPtr child1 = state1->getChild();
		WorldState::ConstPtr child2 = state1->getChild();

		std::deque<WorldState::ConstPtr> validStates;
		validStates.push_back(state1);

		WorldState::ConstPtr ancestor = child1->findCommonAncestor(validStates, child2);
		ASSERT_TRUE(ancestor->getId() == state1->getId());


		WorldState::ConstPtr child1_4 = child1->getChild()->getChild()->getChild()->getChild();
		WorldState::ConstPtr child2_2 = child2->getChild();

		ancestor = child1_4->findCommonAncestor(validStates, child2_2);
		ASSERT_TRUE(ancestor == state1);

	}
	
	TEST_F(WorldModelTest, testfindCommonAncestorNotOldest) {
		WorldState::ConstPtr state1(new WorldState(NULL));

		WorldState::ConstPtr ancestor = state1->getChild();
		WorldState::ConstPtr child1 = ancestor->getChild();
		WorldState::ConstPtr child2 = ancestor->getChild();
		
		std::deque<WorldState::ConstPtr> validStates;
		validStates.push_back(state1);
		validStates.push_back(ancestor);

		WorldState::ConstPtr commonParent = child1->findCommonAncestor(validStates, child2);
		ASSERT_TRUE(commonParent->getId() == ancestor->getId());
	}
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
