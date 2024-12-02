#include "lemlib/api.hpp"
#include "main.h"

void match();
void skills();

extern void (*autonFunctions[])();

extern int autonSelect;
extern std::string autonNames[];

void previousAuton();
void nextAuton();