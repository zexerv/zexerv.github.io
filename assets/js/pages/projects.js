// assets/js/pages/projects.js

document.addEventListener('DOMContentLoaded', () => {
    const filterButtons = document.querySelectorAll('.btn-filter');
    const projectCards = document.querySelectorAll('.project-card');

    if (filterButtons.length > 0 && projectCards.length > 0) {
        filterButtons.forEach(button => {
            button.addEventListener('click', () => {
                const filterValue = button.getAttribute('data-filter');

                // Update active button state
                filterButtons.forEach(btn => btn.classList.remove('active'));
                button.classList.add('active');

                // Filter cards
                projectCards.forEach(card => {
                    const cardCategories = card.getAttribute('data-category').split(' ');

                    if (filterValue === 'all' || cardCategories.includes(filterValue)) {
                        // card.style.display = 'flex'; // Or 'block' depending on original display
                        card.classList.remove('hidden');
                    } else {
                        // card.style.display = 'none';
                         card.classList.add('hidden');
                    }
                });
            });
        });

        // Optional: Initially trigger the 'all' filter if needed
        // document.querySelector('.btn-filter[data-filter="all"]').click();
    }
});