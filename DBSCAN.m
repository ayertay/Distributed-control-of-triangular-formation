%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPML110
% Project Title: Implementation of DBSCAN Clustering in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function [IDX, isnoise, D]=DBSCAN(X,epsilon,MinPts)

    C=0;
    
    n=size(X,1);
    IDX=zeros(n,1);
    
    %D=pdist2(X,X)
    for (i=1:1:5)
        for (j=1:1:6)
            D(i,j) = sqrt((X(n+j-6,1) - X(i,1))^2 + (X(n+j-6,2) - X(i,2))^2);
        end
        for (j=7:1:11)
            D(i,j) = sqrt((X(j-6,1) - X(i,1))^2 + (X(j-6,2) - X(i,2))^2);
        end
    end
    for (i=6:1:n-5)
        for (j=1:1:11)
            D(i,j) = sqrt((X(i+j-6,1) - X(i,1))^2 + (X(i+j-6,2) - X(i,2))^2);
        end 
    end
    for (i=n-4:1:n)
        for (j=1:1:6)
            D(i,j) = sqrt((X(i+j-6,1) - X(i,1))^2 + (X(i+j-6,2) - X(i,2))^2);
        end
        for (j=7:1:11)
            D(i,j) = sqrt((X(j-6,1) - X(i,1))^2 + (X(j-6,2) - X(i,2))^2);
        end
    end
    length(D)
    S = whos('D');
    varSize = S.bytes
    
    visited=false(n,1);
    isnoise=false(n,1);
    
    for i=1:n
        if ~visited(i)
            visited(i)=true;
            
            Neighbors=RegionQuery(i);
            if length(Neighbors)<MinPts
                % X(i,:) is NOISE
                isnoise(i)=true;
            else
                C=C+1;
                ExpandCluster(i,Neighbors,C, n);
            end
            
        end
    
    end
    
    function ExpandCluster(i,Neighbors,C, n)
        IDX(i)=C;
        
        k = 1;
        while true
%             if (i >= 6) %%%%%%%%%%%%%%%%
%                 j = Neighbors(k) + i - 6;
%             else
%                 if ( Neighbors(k) >= 6)
%                     j = Neighbors(k) + i - 6;
%                 else
%                     j = Neighbors(k) + n - 6;
%                 end
%             end
                if (((Neighbors(k) + i - 6) > 0) && ((Neighbors(k) + i - 6) <= n))
                    j = Neighbors(k) + i - 6;
                elseif ((Neighbors(k) + i - 6) > n)
                    j = Neighbors(k) + i - 6 - n;
                else
                    j = Neighbors(k) + i - 6 + n;
                end
                        
                    
            
            if ~visited(j)
                visited(j)=true;
                Neighbors2=RegionQuery(j);

                %if (i > 6)%%%%%%%%%%%%%%%%%%%%
                for ctr_n = 1:1:length(Neighbors2)
                    if (Neighbors2(ctr_n) + (j - i) <= 0)
                        Neighbors2(ctr_n) = Neighbors2(ctr_n) + (j - i) + n;
                    elseif (Neighbors2(ctr_n) + (j - i) > n)
                        Neighbors2(ctr_n) = Neighbors2(ctr_n) + (j - i) - n;
                    else
                        Neighbors2(ctr_n) = Neighbors2(ctr_n) + (j - i);
                    end
                end
                  %Neighbors2 = Neighbors2 + (j - i)
                %end
                if length(Neighbors2)>=MinPts
                    Neighbors=[Neighbors Neighbors2];   %#ok
                end
            end

%             if (i > 6) %%%%%%%%%%%%%%%%
%                 if IDX(j + i - 6)==0
%                     IDX(j + i - 6)=C;
%                 end
%             else
             if IDX(j)==0
                IDX(j)=C;
             end
            % end
            
            k = k + 1;
            if k > length(Neighbors)
                break;
            end
        end
    end
    
    function Neighbors=RegionQuery(i)
        Neighbors=find(D(i,:)<=epsilon);
    end

end



