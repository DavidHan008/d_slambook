#include <iostream>
#include<algorithm>
using namespace std; 
void merge_sort(int arr_[],int l,int r );
void guibing(int arr_[],int l,int mid,int r);
void fast_pre(int arr_[],int l_min,int r_max );
int fast(int arr_[],int l_min,int r_max);
int main( int argc, char** argv )
{

//    int arr[4]={4,3,1,2};
// 	for(int i =1;i<5;i++)
// 	{
// 		for(int j=1;j<=i;j++)
// 		{
// 		      	if(arr[j]<arr[j-1])
// 				swap(arr[j-1],arr[j]);
// 		}
// 	}
//    std::cout<<"arr:"<<arr[0]<<arr[1]<<arr[2]<<arr[3]<<std::endl;
//     int arr2[4]={4,3,1,2};

//    for(int i =1;i<4;i++)
//    {
// 	   for (int j=i;j>0;j--)
// 	   {
// 		  	if(arr2[j]<arr2[j-1])
// 				swap(arr2[j-1],arr2[j]);   
// 	   }
//    }
//      std::cout<<"arr2:"<<arr2[0]<<arr2[1]<<arr2[2]<<arr2[3]<<std::endl;
	
// 	 int arr3[4]={4,3,1,2};

//    for(int i =1;i<4;i++)
//    {
// 	   int arr_temp=arr3[i];
// 	   int j;
// 	   for( j=i;j>=0&&arr3[j-1]>arr_temp;j--)
// 	   {
// 		   arr3[j]=arr3[j-1];
// 	   }
// 	   arr3[j]=arr_temp;
//    }
//      std::cout<<"arr:"<<arr3[0]<<arr3[1]<<arr3[2]<<arr3[3]<<std::endl;
// 	int arr4[8]={8,6,2,3,1,5,7,4};
// 	int num_l,num_r;
// 	int arr4l[4]={0};
// 	int arr4r[4]={0};
// 	//输出正确的数据分类结果
// 	merge_sort(arr4,0,7);
// 	std::cout<<"arr:"<<arr4[0]<<arr4[1]<<arr4[2]<<arr4[3]<<std::endl;
//  	std::cout<<"arr:"<<arr4[4]<<arr4[5]<<arr4[6]<<arr4[7]<<std::endl;
	 //快速排序
	 int arr5[8]={4,2,1,6,8,5,3,7};
	 fast_pre(arr5,0,7);
	std::cout<<"arr:"<<arr5[0]<<arr5[1]<<arr5[2]<<arr5[3]<<std::endl;
 	std::cout<<"arr:"<<arr5[4]<<arr5[5]<<arr5[6]<<arr5[7]<<std::endl;

   return 0;
}



void fast_pre(int arr_[],int l_min,int r_max )
{
	if(l_min+1>=r_max)
	return;
	int p=fast(arr_,l_min,r_max);
	std::cout<<"p"<<p<<std::endl;
	fast_pre(arr_,l_min,p-1);//只是对后面的元素进行赋值
 	fast_pre(arr_,p,r_max);
}

int fast(int arr_[],int l_min,int r_max )
{
	int i_index=l_min+1;
	int j_index;
	//int a_first=arr_[0];
	for(j_index=1;j_index<r_max+1;j_index++)
	{
		if(arr_[j_index]<arr_[0])
		{
			std::cout<<"a------->"<<arr_[j_index]<<std::endl;
			std::cout<<"index_now"<<i_index<<std::endl;
			if(j_index!=i_index)
			{
				swap(arr_[j_index],arr_[i_index]);
			}
				i_index++;	
		}
	}
	swap(arr_[i_index-1],arr_[0]);
	return i_index-1;
}

void merge_sort(int arr_[],int l,int r )
{
	if(l>=r)
	return;//循环终止
	int mid=(r+l+1)/2;
	merge_sort(arr_,l,mid-1);
	merge_sort(arr_,mid,r);
	guibing(arr_,l,mid,r);
}
void guibing(int arr_[],int l,int mid,int r)
{
	if(mid-l==2|| r-mid==2)
	{
		if(arr_[mid-1]<arr_[l])
		{
			swap(arr_[mid-1],arr_[l]);
		}
		if(arr_[mid]>arr_[r])
		{
			swap(arr_[mid],arr_[r]);
		}
	}
	// else
	// {
	// 	/* code */
	// //首先申请一个内存空间
    //  	int arr_temp[2*mid];
	// 	 //给新数组进行赋值
	// 	 for(int i=0;i<2*mid;i++)
	// 	 {
	// 		 arr_temp[i]=arr_[i];
	// 	 }
	// 	 int r_ptr,l_ptr,mid_ptr;
	// 	 r_ptr=r;
	// 	 l_ptr=l;
	// 	 mid_ptr=mid;
	// 	 for (int k=0;k<mid;k++)
	// 	 {
	// 		 //考虑一些极限的情况
	// 		 //总共会出现三种情况
	// 		 if(mid_ptr>r_ptr)
	// 		 {
	// 			 arr_[k]=arr_temp[l_ptr];
	// 			 l_ptr++;
	// 		 }
	// 		 else if (l_ptr>mid_ptr-1)
	// 		 {
	// 			 /* code */
	// 			 arr_[k]=arr_temp[mid_ptr];
	// 			 mid_ptr++;
	// 		 }
	// 		 else
	// 		 {
	// 			 //当mid 和 l 和 r都是有效的时候
	// 			if (arr_temp[mid_ptr]<arr_temp[l_ptr])
	// 				{
	// 					arr_[k]=arr_temp[mid_ptr];
	// 					mid_ptr++;
	// 				}
	// 				else
	// 				{
	// 					arr_[k]=arr_temp[l_ptr];
	// 					l_ptr++;
	// 				}
	// 		 }
	// 	 }
	// }
}

//step 1 分割
//step 数据排序
//step 最后的归并