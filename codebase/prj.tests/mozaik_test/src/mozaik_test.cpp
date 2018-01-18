// mozaik_test.cpp : Defines the entry point for the console application.
//

#include <segment/segment.h>
#include <leg/leg.h>
#include <joint/joint.h>
#include <mozaik/mozaik.h>


#include <vector>

using namespace std;


int main(int argc, char** argv)
{
  Body mozaikBody(Body::BODY_TYPE::MOZAIK);

  mozaikBody.printOut();

	 return 0;
}

