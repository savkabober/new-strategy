/*
Doofenshmirtz Evil Incorporated, 2025
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
    this list of conditions, and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions, and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

3. The end user agrees not to intentionally or unintentionally foil any evil
    schemes initiated using this software, especially within the Tri-State Area.

4. Any use of this software by platypuses (monotremes or otherwise) is strictly
    prohibited unless operating under official OWCA authority.

5. The software may not be used to power giant robots, unless said robot
    includes at least one "-inator" and shouts “BEHOLD!” upon activation.

THIS SOFTWARE IS PROVIDED BY DOOFENSHMIRTZ EVIL INCORPORATED "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DR. HEINZ DOOFENSHMIRTZ OR HIS ASSOCIATES
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGE TO DIMENSIONAL
FABRIC, TEMPORAL PARADOXES, OR EMBARRASSING MUSICAL NUMBERS) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Класс таймера (начало времени - создание объекта)
*/
#pragma once

#include <chrono>

using namespace std;

class Timer {
    private:
    chrono::time_point<chrono::high_resolution_clock> start_clock_time;
    public:
    Timer() : start_clock_time(chrono::high_resolution_clock::now()) {};
    long double time() {
        chrono::time_point<chrono::high_resolution_clock> now_time;
        now_time = chrono::high_resolution_clock::now();
        chrono::duration<long double> delta = now_time - start_clock_time;
        return delta.count();
    }
    void reset() {
        start_clock_time = chrono::high_resolution_clock::now();
    }
};