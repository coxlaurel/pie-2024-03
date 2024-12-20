document.querySelectorAll('.shape').forEach(shape => {
    const size = shape.getAttribute('data-size');
    const posX = shape.getAttribute('data-position-x') || '0';
    const posY = shape.getAttribute('data-position-y') || '0';
    const opacity = shape.getAttribute('opacity') || '1';

    if (size.includes('x')) {
        const [width, height] = size.split('x');
        shape.style.setProperty('--width', `${width}px`);
        shape.style.setProperty('--height', `${height}px`);
    } else {
        shape.style.setProperty('--width', `${size}px`);
        shape.style.setProperty('--height', `${size}px`);
    }

    shape.style.setProperty('--pos-x', `${posX}px`);
    shape.style.setProperty('--pos-y', `${posY}px`);
    shape.style.setProperty('--opacity', opacity);
});
