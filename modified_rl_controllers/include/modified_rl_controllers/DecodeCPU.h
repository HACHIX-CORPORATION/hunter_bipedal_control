#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
#include <arpa/inet.h>
#include <unistd.h>
#include <onnxruntime/onnxruntime_cxx_api.h>

int tran(char& ch);

void swap_u16_values(std::vector<uint16_t> &data, const std::vector<uint16_t> &byte_array);

std::string executeCommand(const std::string& command);

std::vector<uint16_t> hex_string_to_byte_array(std::string& hex_str);

std::string getCpuId();

std::vector<char> Decode(const char* path);