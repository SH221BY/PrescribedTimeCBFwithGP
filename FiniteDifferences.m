function Dx = FiniteDifferences(X,dt)
    Len = size(X,2);
    Dim = size(X,1);
    Dx = zeros(Dim, Len);
    for i = 2:Len-1
        for j = 1:Dim
            Dx(j,i)=(-X(j,i-1)+X(j,i+1))/(dt(i+1)-dt(i-1));
        end
    end
end