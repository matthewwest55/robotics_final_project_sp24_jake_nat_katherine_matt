import random

class Hangman(object):
    def __init__(self, wordlist, guesses_allowed):
        self.guesses = []
        self.secret_word = self.choose_word(wordlist)
        print(self.secret_word)
        self.remaining_guesses = guesses_allowed
        while not self.is_game_over():
            print("Word:", self.display_word())
            print("Attempts remaining:", self.remaining_guesses)
            guess = input("Enter a letter: ")
            self.guess_letter(guess)

    def choose_word(self, wordlist):
        return random.choice(wordlist)

    def display_word(self):
        displayed_word = ''
        for letter in self.secret_word:
            if letter in self.guesses:
                displayed_word += letter + ' '
            else:
                displayed_word += '_ '
        return displayed_word.strip()
    
    def guess_letter(self, letter):
        letter = letter.upper()
        if letter in self.guesses:
            print("Already Guessed.")
        elif letter in self.secret_word:
            print("Good guess!")
            # repeat guesses already handled by cam file
            self.guesses.append(letter)
            return True
        else:
            print("Wrong guess!")
            self.remaining_guesses -= 1
            self.guesses.append(letter)
            return False
    

    def is_game_over(self):
        if self.remaining_guesses <= 0:
            print("Out of attempts. The word was:", self.secret_word)
            return True
        elif '_' not in self.display_word():
            print("You guessed:", self.secret_word)
            return True
        else:
            return False
