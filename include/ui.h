#pragma once
#include <stdint.h>
void uiInit();
void uiPrintLines(const char* l1="", const char* l2="", const char* l3="", const char* l4="");
// Zeige ein Boot-Logo (nicht blockierend außer kurzer Anzeigezeit intern)
void uiShowBootLogo();
