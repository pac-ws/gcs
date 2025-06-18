#include <ncurses.h>

class TableColumnBase {
    public:
        virtual ~TableColumnBase() = default;
        virtual void DrawColumn(int col_start) = 0;
        virtual void ClearData() = 0;
        virtual size_t GetSize() const = 0;
        virtual bool IsEmpty() const = 0;
        virtual int GetWidth() const = 0;
        virtual const std::string& GetHeader() const = 0;
};

template<typename T>
class TableColumn : public TableColumnBase {
    WINDOW* table_;
    std::string header_;
    std::vector<T> data_;
    int width_;
    int row_start_;

    public:
        TableColumn(WINDOW* table, const std::string& header, const std::vector<T>& data, int width, int row_start)
            : table_(table), header_(header), data_(data), width_(width), row_start_(row_start) {}

        void SetData(const std::vector<T>& data) {
            data_ = data;
        }

        void ClearData() override {
            data_.clear();
        }

        void AddRow(const T& entry) {
            data_.push_back(entry);
        }

        size_t GetSize() const override {
            return data_.size();
        }

        bool IsEmpty() const override {
            return data_.empty();
        }

        const std::vector<T>& GetData() const {
            return data_;
        }

        int GetWidth() const override{
            return width_;
        }

        const std::string& GetHeader() const override{
            return header_;
        }

        void DrawColumn(int col_start) override{
            mvwprintw(table_, row_start_, col_start, header_.c_str());
            size_t row = row_start_ + 1;
            for (const auto& item : data_){
                if (row > getmaxy(table_)) {
                    break;
                }
                std::string str_item = ConvertToString(item);
                mvwprintw(table_, row, col_start, "%.*s", str_item.c_str());
                row++;
            }
        }

    private:
        std::string ConvertToString(const T& item) {
            std::ostringstream oss;
            oss << item;
            return oss.str();
        }
};
