/*
 * Project music
 * Description:
 * Author:
 * Date:
 */
#include <iostream>
#include <windows.h>
#include "MMSystem.h"

using nbamespace stdp;
int main()
{
  PlaySound(TEXT("Sela.wav"), NULL,SND_SYNC);
  system("pause");
  return 0;
}

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.

}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.

}