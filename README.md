The code in both Skele-Files is, as of now, just using the API calls from the MMS (the Micro Mouse Simulator). The code is, thus, not suitable for the actual robot in its current form, because it has no way of sensing its surroundings.

Luckily, this is an easy fix. We'll just have to replace the API inputs to those of our IR-sensors. We'll also have to add in direction control from the gyroscope sensors – as the code doesn't have one of its own due to MMS forcing one.

"Penalize" primarily penalizes the mouse from potentialy backtracking and looping – the 4 P:s. It's better for a more open-space maze, but horrendously shitty when the corridors are more long-winded. 
 
 "Encouraging" encourages a more goal-oriented approach rather than the exploratory one in "Penalize". Infortunatly, it has the habit of investigating every nook and crany. Though, the err could be remedied by combining elements of "Penalize" and "Encouraging" into one "amalgamated" code. I'll call the file of this amalgamated code "Skele-file_Amalg" – truely the best name there ever was. 

"Encouraging" is better in most regards except in the "ALLJAPAN" maps – where "Penalize" reaches the goal in a shorter distance.
"Smartass" was a bit jittery at times - though this should be fixed now. 
"Smartass" chooses the shorter route most of the time; maze 3 is an anomaly – maybe if we store a total distance value for both paths a&b and if a<'b the robot would choose a (potential solution).
I made the code on the supposition that the competition revolved around spontaneous maze-solving. if there's a explore–and–return–to–start element we can add a second phase. [Already added] 
         






