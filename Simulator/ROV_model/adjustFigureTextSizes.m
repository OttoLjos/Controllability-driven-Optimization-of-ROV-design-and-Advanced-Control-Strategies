function adjustFigureTextSizes(titleSize, labelSize, tickSize)
    % Adjust text sizes of a figure
    % fig: handle to the figure
    % titleSize: font size for the title
    % labelSize: font size for the axis labels
    % tickSize: font size for the axis ticks

    figs = findall(0, 'Type', 'figure');
    for idx = 1:length(figs)
        fig = figs(idx);
    
        axes = findall(fig, 'type', 'axes');
        for ax = axes'
            set(get(ax, 'XLabel'), 'FontSize', labelSize);
            set(get(ax, 'YLabel'), 'FontSize', labelSize);
            set(get(ax, 'Title'), 'FontSize', titleSize);
            set(ax, 'FontSize', tickSize);
        end
    end
end