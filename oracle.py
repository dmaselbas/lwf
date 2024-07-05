import streamlit as st

def main():
    st.title("Streamlit App")
    user_input = st.text_input("Enter some text:")
    submit_button = st.button("Submit")

    if submit_button:
        st.write(f"You entered: {user_input}")

if __name__ == "__main__":
    main()
