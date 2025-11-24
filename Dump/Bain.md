Main.c was renamed Bain.c to avoid compiler main duplication errors.

To use a specific "main" ensure that no other iterations of a main.c exist, or change all references in solver.h, solver.c, etc. to fit the chosen file – in this case, "Bain.c". 

And yes, I'm aware that one could just delete these files – they're redundant any way – , but it's neat piece of history. 

Place the file in the same place as its included dependences.