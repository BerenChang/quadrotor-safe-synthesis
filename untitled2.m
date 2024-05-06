Xvals = [1,2,3,4,5,6,7,8,9];
Yvals = [2,5,5,5,2,2,5,5,5];

cla()
hold on
isGreater = Yvals > 3; 
scatter(Xvals(~isGreater), Yvals(~isGreater), 50, 'k', 'filled')
scatter(Xvals(isGreater), Yvals(isGreater), 50, 'r', 'filled')