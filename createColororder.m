function newColororder = createColororder(num)
    newColororder = zeros(num,3)*0.8+0.2;
    for i=1:num
        new_rgb = rand(1,3);
        %�����ɺڰ׻�ɫ
        if new_rgb(1)==new_rgb(2)==new_rgb(3)
            i = i-1;
            continue;
        end
        %�����ɺ�ɫ
        if sum(new_rgb==[1 0 0])==3
            i = i-1;
            continue;
        end
        %�������ظ���ɫ
        is_repeat = false;
        for j=1:i-1
            if(sum(new_rgb==newColororder(j,:))==3)
                is_repeat=true;
                break;
            end
        end
        if is_repeat
            i = i-1;
            continue;
        end
        newColororder(i,:) = new_rgb;
    end
end