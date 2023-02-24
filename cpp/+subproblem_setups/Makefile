all:
	echo "Timing tests for all subproblems"
	echo "Timing test results: \n" > timing.txt
	
	echo "subproblem 1: " >> timing.txt
	g++ -Wall sp_1.cpp -o a.out -O3
	./a.out >> timing.txt
	echo "" >> timing.txt
	
	echo "subproblem 2: " >> timing.txt
	g++ -Wall sp_2.cpp -o a.out -O3
	./a.out >> timing.txt
	echo "" >> timing.txt

	echo "subproblem 2 Extended: " >> timing.txt
	g++ -Wall sp_2E.cpp -o a.out -O3
	./a.out >> timing.txt
	echo "" >> timing.txt

	echo "subproblem 3: " >> timing.txt
	g++ -Wall sp_3.cpp -o a.out -O3
	./a.out >> timing.txt
	echo "" >> timing.txt

	echo "subproblem 4: " >> timing.txt
	g++ -Wall sp_4.cpp -o a.out -O3
	./a.out >> timing.txt
	echo "" >> timing.txt

	echo "subproblem 5: " >> timing.txt
	g++ -Wall sp_5.cpp -o a.out -O3
	./a.out >> timing.txt
	echo "" >> timing.txt

	echo "subproblem 6: " >> timing.txt
	g++ -Wall sp_6.cpp -o a.out -O3
	./a.out >> timing.txt
	echo "" >> timing.txt

	echo "\nresults are recorded in timing.txt\n"