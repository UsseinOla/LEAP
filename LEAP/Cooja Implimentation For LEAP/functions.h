/*****************************matrix transpose****************************/
float** matrix_transpose(float **M,int L,int C){
    int i,j; float **M_transpose;
    M_transpose=(float**)malloc(C*sizeof(float*));
    for (i=0;i<C;i++)
        M_transpose[i]=(float*)malloc(L*sizeof(float));

    for(i=0;i<C;i++)
        for(j=0;j<L;j++){
            *(*(M_transpose+i)+j)=*(*(M+j)+i);
    }
    return M_transpose;
}
/*************************matrix multiplication**************************/
float** matrix_multiplication(float **M1,int L1,int C1, float **M2,int C2){
    int i,j,k,L2=C1; float **M,s;
    M= (float**)malloc(L1 * sizeof(float*));
    for (i = 0; i < L1; i++)
        M[i] = (float*)malloc(C2 * sizeof(float));

    for(i=0;i<L1;i++) {
        for(j=0;j<C2;j++) {
            s=0;k=0;
            do {
                s+=(*(*(M1+i)+k))*(*(*(M2+k)+j));
                k++;
            }while(k<L2);
        *(*(M+i)+j)=s;
        }
    }
    return M;
}
/*****************************matrix inverse*****************************/
float** matrix_inverse(float **M,int N){
    int i, j, k;float ratio,a,**M_inverse,**aux;
    M_inverse= (float**)malloc(N * sizeof(float*));
    aux= (float**)malloc(N * sizeof(float*));
    for (i = 0; i < N; i++){
        M_inverse[i] = (float*)malloc(N * sizeof(float));
        aux[i] = (float*)malloc(N * sizeof(float));
    }
    for(i=0;i<N;i++)
        for(j=0;j<N;j++)
            *(*(M_inverse+i)+j)=*(*(M+i)+j);
    for(i = 0; i < N; i++){
        for(j = N; j < 2*N; j++){
            if(i==(j-N))
                *(*(M_inverse+i)+j)= 1.0;
            else
                *(*(M_inverse+i)+j)= 0.0;
        }
    }
    for(i=0; i < N; i++){
        for(j = 0; j < N; j++){
            if(i!=j){
                ratio = *(*(M_inverse+j)+i)/(*(*(M_inverse+i)+i));
                for(k = 0; k < 2*N; k++){
                    *(*(M_inverse+j)+k) -= ratio * (*(*(M_inverse+i)+k));
                }
            }
        }
    }
    for(i = 0; i < N; i++){
        a = *(*(M_inverse+i)+i);
        for(j = 0; j < 2*N; j++){
            *(*(M_inverse+i)+j) /= a;
        }
    }

    for(i=0;i<N;i++){
        k=0;
        for(j=N;j<2*N;j++){
            *(*(aux+i)+k)=*(*(M_inverse+i)+j);
            k++;
        }
    }
    return aux;
}
