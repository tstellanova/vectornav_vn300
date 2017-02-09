#include <unit_test/unit_test.h>

#include <systemlib/err.h>
#include <stdio.h>
#include <unistd.h>

extern "C" __EXPORT int vn300_tests_main(int argc, char *argv[]);

class VectorNav_VN300Test : public UnitTest
{
public:
	virtual bool run_tests(void);

private:
	bool vn300Test();
};

bool VectorNav_VN300Test::run_tests(void)
{
	ut_run_test(vn300Test);

	return (_tests_failed == 0);
}

bool VectorNav_VN300Test::vn300Test(void)
{



	return false; //TODO implement test and pass
}

ut_declare_test_c(vn300_tests_main, VectorNav_VN300Test)

