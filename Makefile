CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

run:
# g++ all_files_with.cpp -o  directory/main.out
# then execute ./directory/main.out
 
	$(CXX) $(CXXFLAGS) $(DIR)/*.cpp -o $(DIR)/main.out
	./$(DIR)/main.out