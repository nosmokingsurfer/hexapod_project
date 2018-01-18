// articulated_test.cpp : Defines the entry point for the console application.
//


#include <body/body.h>


#include <vector>

using namespace std;


int main(int argc, char** argv)
{
  Body articul(Body::BODY_TYPE::ARTICULATED);

  articul.printOut();
	 return 0;
}

