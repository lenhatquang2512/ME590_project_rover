#include <iostream>
#include <vector>

template <typename S>
inline void swap(S &a, S &b){
	S temp;
	temp = a;
	a = b;
	b = temp;
}

template <typename S>
void bubbleSort(std::vector<S> &v){

	int n = v.size();
	bool swapped ;

	for (int i = 0; i < n; ++i)
	{
		swapped = false;
		for (int j = i; j < n; ++j)
		{
			if (v[i] > v[j])
			{
				swap<S>(v[i],v[j]);
				swapped = true;
			}
		}

		if (swapped == false)
			break;
	}

}

template <typename S>
void printArray(std::vector<S> *v)
{
	for (int i = 0; i < v->size(); i++) {
		std::cout << (*v)[i] << " " ;
    }

    std::cout << std::endl;
}

int main(int argc, char const *argv[])
{

	std::vector<int> arr{ 64, 34, 25, 12, 22, 11, 90 };

	std::cout << "Before Sorted array: \n";
    printArray<int>(&arr);

    bubbleSort<int>(arr);

    std::cout << "After Sorted array: \n";
    printArray<int>(&arr);

	return 0;
}