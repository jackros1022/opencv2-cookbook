#include <iostream>
#include <vector>

class AClass {
public:
    int id;
    double value;
    double x, y;
};

void vector_iter()
{
    int array[10];
    
    int *begin = array;
    int *end = array + 10;
    printf ("int array[] = ");
    for (int *it = begin; it!=end; ++it)
        printf ("%d ", *it);
    printf ("\n");

    std::vector<int> v(10);
    printf ("std::vector array[] = ");
    for (std::vector<int>::iterator it=v.begin(); it!=v.end(); ++it)
        printf ("%d ", *it);
    printf ("\n");
    
    std::vector<AClass> vcs(5);
    for (int i=0; i<vcs.size(); i++)
    {
        vcs[i].id = i;
        vcs[i].value = i*10;
    }
    
    printf ("std::vector<AClass> array[] = \n");
    for (std::vector<AClass>::iterator i=vcs.begin(); i != vcs.end(); ++i)
        printf ("%d: value= %lf\n", i->id, i->value);
    printf ("\n");
}

int main(int argc, char *argv[])
{
	vector_iter();
	return 0;
}
// EOF //
