document.querySelectorAll(".quiz").forEach((quiz) => {
  quiz.querySelector(".quiz-check").addEventListener("click", () => {
    const answer = quiz.querySelector("input:checked");
    const result = quiz.querySelector(".quiz-result");

    if (!answer) {
      result.className = "quiz-result";
      result.textContent = "Choose an answer first.";
      return;
    }

    const correct = answer.value === quiz.dataset.answer;
    const correctLabel = quiz.querySelector(
      `input[value="${quiz.dataset.answer}"]`,
    ).parentElement.cloneNode(true);
    correctLabel.querySelector("input").remove();
    result.className = `quiz-result ${correct ? "quiz-correct" : "quiz-incorrect"}`;
    result.innerHTML = correct
      ? `Correct. ${quiz.dataset.explanation}`
      : `Not quite. Correct answer: ${correctLabel.innerHTML.trim()}. ${quiz.dataset.explanation}`;
  });
});
