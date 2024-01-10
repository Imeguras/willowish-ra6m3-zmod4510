#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <nlohmann/json.hpp>



int main(int argc, char **argv){
	nlohmann::json json;
	json["value"] = 1;
	
	//convert it to a string
	std::string json_string = json.dump();
	std::cout << json_string << std::endl;
	return 0;
}